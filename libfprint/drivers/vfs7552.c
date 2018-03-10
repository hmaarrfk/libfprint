/*
 * Validity Sensors, Inc. VFS7552 Fingerprint Reader driver for libfprint
 *                        ID 138a:0091 Validity Sensors, Inc.
 * Copyright (C) 2013 Mark Harfouche <mark.harfouche@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/* A large chunk of this code is based on a project aimed at simply
 * getting images from the sensor.
 *
 * Find the project at
 * https://github.com/hmaarrfk/Validity91
 */

/* Mark's personal notes:
  the skeleton for this was taken from 5011.
  I realize now that it is a swiping sensor, which might not be the best one to emulate
  The issue with swiping sensors is that they require some kind of stitching between lines
  Therefore, the algorithm needs to be clever in how it puts the final image together.

  The press type device is thus much easier to use.
  It might be better to emulate FP_SCAN_TYPE_PRESS devices

  I think it might be time to look at the uru4000 for more understanding
  on how a touch sensor works
*/

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <libusb.h>
#include <fp_internal.h>
#include <assembling.h>
#include "driver_ids.h"

#include "vfs7552_proto.h"

#define VFS7552_CONTROL_PIXELS (8)
#define VFS7552_LINE_SIZE (VFS7552_IMAGE_WIDTH + VFS7552_CONTROL_PIXELS)

/* =================== sync/async USB transfer sequence ==================== */

enum {
	ACTION_SEND,
	ACTION_RECEIVE,
};

struct usb_action {
	int type;
	const char *name;
	int endpoint;
	int size;
	unsigned char *data;
	int correct_reply_size;
};

#define SEND(ENDPOINT, COMMAND) \
{ \
	.type = ACTION_SEND, \
	.endpoint = ENDPOINT, \
	.name = #COMMAND, \
	.size = sizeof(COMMAND), \
	.data = COMMAND, \
  .correct_reply_size = 0 \
},

#define RECV(ENDPOINT, SIZE) \
{ \
	.type = ACTION_RECEIVE, \
	.endpoint = ENDPOINT, \
	.size = SIZE, \
	.data = NULL, \
  .correct_reply_size = 0, \
},

#define RECV_CHECK(ENDPOINT, SIZE, EXPECTED) \
{ \
	.type = ACTION_RECEIVE, \
	.endpoint = ENDPOINT, \
	.size = SIZE, \
	.data = EXPECTED, \
	.correct_reply_size = sizeof(EXPECTED) \
},
#define RECV_CHECK_SIZE(ENDPOINT, SIZE, EXPECTED) \
{ \
	.type = ACTION_RECEIVE, \
	.endpoint = ENDPOINT, \
	.size = SIZE, \
	.data = NULL, \
	.correct_reply_size = sizeof(EXPECTED) \
},

struct usbexchange_data {
	int stepcount;
	struct fp_img_dev *device;
	struct usb_action *actions;
	void *receive_buf;
	int timeout;
};


static void async_send_cb(struct libusb_transfer *transfer)
{
	struct fpi_ssm *ssm = transfer->user_data;
	struct usbexchange_data *data = (struct usbexchange_data *)ssm->priv;
	struct usb_action *action;

	if (ssm->cur_state >= data->stepcount) {
		fp_err("Radiation detected!");
		fpi_imgdev_session_error(data->device, -EINVAL);
		fpi_ssm_mark_aborted(ssm, -EINVAL);
		goto out;
	}

	action = &data->actions[ssm->cur_state];
	if (action->type != ACTION_SEND) {
		fp_err("Radiation detected!");
		fpi_imgdev_session_error(data->device, -EINVAL);
		fpi_ssm_mark_aborted(ssm, -EINVAL);
		goto out;
	}

	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		/* Transfer not completed, return IO error */
		fp_err("transfer not completed, status = %d", transfer->status);
		fpi_imgdev_session_error(data->device, -EIO);
		fpi_ssm_mark_aborted(ssm, -EIO);
		goto out;
	}
	if (transfer->length != transfer->actual_length) {
		/* Data sended mismatch with expected, return protocol error */
		fp_err("length mismatch, got %d, expected %d",
			transfer->actual_length, transfer->length);
		fpi_imgdev_session_error(data->device, -EIO);
		fpi_ssm_mark_aborted(ssm, -EIO);
		goto out;
	}

	/* success */
	fpi_ssm_next_state(ssm);

out:
	libusb_free_transfer(transfer);
}

static void async_recv_cb(struct libusb_transfer *transfer)
{
	struct fpi_ssm *ssm = transfer->user_data;
	struct usbexchange_data *data = (struct usbexchange_data *)ssm->priv;
	struct usb_action *action;

	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		/* Transfer not completed, return IO error */
		fp_err("transfer not completed, status = %d", transfer->status);
		fpi_imgdev_session_error(data->device, -EIO);
		fpi_ssm_mark_aborted(ssm, -EIO);
		goto out;
	}

	if (ssm->cur_state >= data->stepcount) {
		fp_err("Radiation detected!");
		fpi_imgdev_session_error(data->device, -EINVAL);
		fpi_ssm_mark_aborted(ssm, -EINVAL);
		goto out;
	}

	action = &data->actions[ssm->cur_state];
	if (action->type != ACTION_RECEIVE) {
		fp_err("Radiation detected!");
		fpi_imgdev_session_error(data->device, -EINVAL);
		fpi_ssm_mark_aborted(ssm, -EINVAL);
		goto out;
	}

  if (action->correct_reply_size != 0) {
    if (transfer->actual_length != action->correct_reply_size) {
      fp_err("Got %d bytes instead of %d",
        transfer->actual_length,
        action->correct_reply_size);
      fpi_imgdev_session_error(data->device, -EIO);
      fpi_ssm_mark_aborted(ssm, -EIO);
      goto out;
    }
  }
  else {
		fp_dbg("Got %d bytes out of %d", transfer->actual_length,
		       transfer->length);
  }

	if (action->data != NULL) {
		if (memcmp(transfer->buffer, action->data,
					action->correct_reply_size) != 0) {
			fp_dbg("Wrong reply: 0x%02x 0x%02x", transfer->buffer[0], transfer->buffer[1]);
			fpi_imgdev_session_error(data->device, -EIO);
			fpi_ssm_mark_aborted(ssm, -EIO);
			goto out;
		}
	}

	fpi_ssm_next_state(ssm);
out:
	libusb_free_transfer(transfer);
}

static void usbexchange_loop(struct fpi_ssm *ssm)
{
	struct usbexchange_data *data = (struct usbexchange_data *)ssm->priv;
	if (ssm->cur_state >= data->stepcount) {
		fp_err("Bug detected: state %d out of range, only %d steps",
				ssm->cur_state, data->stepcount);
		fpi_imgdev_session_error(data->device, -EINVAL);
		fpi_ssm_mark_aborted(ssm, -EINVAL);
		return;
	}

	struct usb_action *action = &data->actions[ssm->cur_state];
	struct libusb_transfer *transfer;
	int ret = -EINVAL;

	switch (action->type) {
	case ACTION_SEND:
		fp_dbg("Sending %s", action->name);
		transfer = libusb_alloc_transfer(0);
		if (transfer == NULL) {
			fp_err("Failed to allocate transfer");
			fpi_imgdev_session_error(data->device, -ENOMEM);
			fpi_ssm_mark_aborted(ssm, -ENOMEM);
			return;
		}
		libusb_fill_bulk_transfer(transfer, data->device->udev,
					  action->endpoint, action->data,
					  action->size, async_send_cb, ssm,
					  data->timeout);
		ret = libusb_submit_transfer(transfer);
		break;

	case ACTION_RECEIVE:
		fp_dbg("Receiving %d bytes", action->size);
		transfer = libusb_alloc_transfer(0);
		if (transfer == NULL) {
			fp_err("Failed to allocate transfer");
			fpi_imgdev_session_error(data->device, -ENOMEM);
			fpi_ssm_mark_aborted(ssm, -ENOMEM);
			return;
		}
		libusb_fill_bulk_transfer(transfer, data->device->udev,
					  action->endpoint, data->receive_buf,
					  action->size, async_recv_cb, ssm,
					  data->timeout);
		ret = libusb_submit_transfer(transfer);
		break;

	default:
		fp_err("Bug detected: invalid action %d", action->type);
		fpi_imgdev_session_error(data->device, -EINVAL);
		fpi_ssm_mark_aborted(ssm, -EINVAL);
		return;
	}

	if (ret != 0) {
		fp_err("USB transfer error: %s", strerror(ret));
		fpi_imgdev_session_error(data->device, ret);
		fpi_ssm_mark_aborted(ssm, ret);
	}
}

static void usb_exchange_async(struct fpi_ssm *ssm,
			       struct usbexchange_data *data)
{
	struct fpi_ssm *subsm = fpi_ssm_new(data->device->dev,
					    usbexchange_loop,
					    data->stepcount);
	subsm->priv = data;
	fpi_ssm_start_subsm(ssm, subsm);
}

/* ====================== main stuff ======================= */

#define VFS7552_IMAGE_CHUNKS (3)

struct vfs7552_data {
	unsigned char *image;
  enum fp_imgdev_state activate_state;
  int image_index;
	unsigned char *capture_buffer;
	int chunks_captured;
	gboolean loop_running;
	gboolean deactivating;
	struct usbexchange_data init_sequence;
	struct libusb_transfer *flying_transfer;
};

enum {
	DEV_ACTIVATE_REQUEST_FPRINT,
	DEV_ACTIVATE_NUM_STATES
};

enum{
  DEACTIVATE_SEND_SHUTDOWN,
  DEACTIVATE_NUM_STATES
};
enum {
  AWAIT_FINGER_ON_INIT,
  AWAIT_FINGER_ON_INTERRUPT_QUERY,
  AWAIT_FINGER_ON_INTERRUPT_CHECK,
  AWAIT_FINGER_ON_NUM_STATES
};

enum {
  CAPTURE_QUERY_DATA_READY,
  CAPTURE_CHECK_DATA_READY,
  CAPTURE_REQUEST_CHUNK,
  CAPTURE_READ_CHUNK,
  CAPTURE_COMPLETE,
  CAPTURE_DISABLE_SENSOR,
  CAPTURE_DISABLE_COMPLETE,
  CAPTURE_NUM_STATES
};
enum {
	DEV_OPEN_START,
	DEV_OPEN_NUM_STATES
};

static void capture_init(struct vfs7552_data *data)
{
	fp_dbg("capture_init");
  data->image_index = 0;
  data->chunks_captured = 0;
}


void report_finger_off(struct fpi_ssm *ssm){
  struct fp_img_dev *dev = (struct fp_img_dev *)ssm->priv;

  fpi_imgdev_report_finger_status(dev, FALSE);
  fpi_ssm_free(ssm);
}
void report_finger_on(struct fpi_ssm *ssm){
  struct fp_img_dev *dev = (struct fp_img_dev *)ssm->priv;

  fpi_imgdev_report_finger_status(dev, TRUE);
  fpi_ssm_free(ssm);
}

void report_deactivate(struct fpi_ssm *ssm){
  struct fp_img_dev *dev = (struct fp_img_dev *)ssm->priv;

  fpi_imgdev_deactivate_complete(dev);
  fpi_ssm_free(ssm);
}

void submit_image(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = (struct fp_img_dev *)ssm->priv;
  struct vfs7552_data *data = (struct vfs7552_data *) dev->priv;
	struct fp_img *img;

	img = fpi_img_new_for_imgdev(dev);
  memcpy(img->data, data->image, VFS7552_IMAGE_SIZE);
	fp_dbg("Image captured, commiting");

	fpi_imgdev_image_captured(dev, img);
  fpi_ssm_free(ssm);
}

enum {
  CHUNK_READ_FINISHED,
  CHUNK_READ_NEED_MORE
};

static int process_chunk(struct vfs7552_data *data, int transferred){
  // TODO
  unsigned char * ptr;
  int n_bytes_in_chunk;
  int n_lines;
  int i;

  ptr = data->capture_buffer;
  n_bytes_in_chunk = ptr[2] + ptr[3] * 256;

  ptr = ptr + 6;
  n_lines = n_bytes_in_chunk / VFS7552_LINE_SIZE;

  for(i = 0; i < n_lines; i++){
    ptr = ptr + 8; // 8 bytes code at the beginning of each line
    memcpy(&data->image[data->image_index], ptr, VFS7552_IMAGE_WIDTH);
    ptr = ptr + VFS7552_IMAGE_WIDTH;
    data->image_index = data->image_index + VFS7552_IMAGE_WIDTH;
  }
  data->chunks_captured = data->chunks_captured + 1;
  if (data->chunks_captured == VFS7552_IMAGE_CHUNKS){
    data->image_index = 0;
    data->chunks_captured = 0;
    return CHUNK_READ_FINISHED;
  }
  return CHUNK_READ_NEED_MORE;
}
static void chunk_capture_callback(struct libusb_transfer *transfer)
{
	struct fpi_ssm *ssm = (struct fpi_ssm *)transfer->user_data;
	struct fp_img_dev *dev = (struct fp_img_dev *)ssm->priv;
	struct vfs7552_data *data = (struct vfs7552_data *)dev->priv;

	if ((transfer->status == LIBUSB_TRANSFER_COMPLETED) ||
	    (transfer->status == LIBUSB_TRANSFER_TIMED_OUT)) {
		if (process_chunk(data, transfer->actual_length) == CHUNK_READ_FINISHED)
      fpi_ssm_next_state(ssm);
		else
			fpi_ssm_jump_to_state(ssm, CAPTURE_REQUEST_CHUNK);
	} else {
		if (!data->deactivating) {
			fp_err("Failed to capture data");
			fpi_ssm_mark_aborted(ssm, -1);
		} else {
			fpi_ssm_mark_completed(ssm);
		}
	}
	libusb_free_transfer(transfer);
	data->flying_transfer = NULL;
}

static int capture_chunk_async(struct vfs7552_data *data,
			       libusb_device_handle *handle,
			       int timeout, struct fpi_ssm *ssm)
{
	data->flying_transfer = libusb_alloc_transfer(0);
	libusb_fill_bulk_transfer(data->flying_transfer, handle,
          VFS7552_IN_ENDPOINT,
				  data->capture_buffer,
				  VFS7552_RECEIVE_BUF_SIZE,
				  chunk_capture_callback, ssm, timeout);
	return libusb_submit_transfer(data->flying_transfer);
}

/*
 *  Device initialization. Windows driver only does it when the device is
 *  plugged in, but it doesn't harm to do this every time before scanning the
 *  image.
 */
struct usb_action vfs7552_initialization[] = {
	SEND(VFS7552_OUT_ENDPOINT, vfs7552_cmd_01)
	RECV_CHECK_SIZE(VFS7552_IN_ENDPOINT, 64, vfs7552_cmd_01_recv)

	SEND(VFS7552_OUT_ENDPOINT, vfs7552_cmd_19)
	RECV_CHECK_SIZE(VFS7552_IN_ENDPOINT, 128, vfs7552_cmd_19_recv)

	SEND(VFS7552_OUT_ENDPOINT, vfs7552_init_00)
	RECV_CHECK(VFS7552_IN_ENDPOINT, 64, VFS7552_NORMAL_REPLY)

	SEND(VFS7552_OUT_ENDPOINT, vfs7552_init_01)
  RECV_CHECK(VFS7552_IN_ENDPOINT, 64, VFS7552_NORMAL_REPLY)

  SEND(VFS7552_OUT_ENDPOINT, vfs7552_init_02)
	RECV_CHECK(VFS7552_IN_ENDPOINT, 64, vfs7552_init_02_recv)

  SEND(VFS7552_OUT_ENDPOINT, vfs7552_init_03)
  RECV_CHECK_SIZE(VFS7552_IN_ENDPOINT, 64, vfs7552_init_03_recv)

  SEND(VFS7552_OUT_ENDPOINT, vfs7552_init_04)
  RECV_CHECK(VFS7552_IN_ENDPOINT, 64, VFS7552_NORMAL_REPLY)
	/*
	 * Windows driver does this and it works
	 * But in this driver this call never returns...
	 * RECV(VFS7552_IN_ENDPOINT_CTRL2, 8)
	 */
};

/* This seems to make the sensor ready to respond on the interrupt
   interface when a finger is placed on the sensor.
 */
struct usb_action vfs7552_stop_capture[] = {
  SEND(VFS7552_OUT_ENDPOINT, vfs7552_cmd_04)
  RECV_CHECK(VFS7552_IN_ENDPOINT, 64, VFS7552_NORMAL_REPLY)

  SEND(VFS7552_OUT_ENDPOINT, vfs7552_cmd_52)
  RECV_CHECK(VFS7552_IN_ENDPOINT, 64, VFS7552_NORMAL_REPLY)
};

/* Initiate recording the image */
struct usb_action vfs7552_initiate_capture[] = {
  SEND(VFS7552_OUT_ENDPOINT, vfs7552_image_start)
  RECV_CHECK_SIZE(VFS7552_IN_ENDPOINT, 2048, vfs7552_image_start_resp)
};

struct usb_action vfs7552_wait_finger_init[] = {
  RECV_CHECK_SIZE(VFS7552_INTERRUPT_ENDPOINT, 8, interrupt_ok)
};
struct usb_action vfs7552_data_ready_query[] = {
  SEND(VFS7552_OUT_ENDPOINT, vfs7552_is_image_ready)
  RECV_CHECK_SIZE(VFS7552_IN_ENDPOINT, 64, vfs7552_is_image_ready_resp_ready)

};
struct usb_action vfs7552_request_chunk[] = {
  SEND(VFS7552_OUT_ENDPOINT, vfs7552_read_image_chunk)
};


/* ====================== lifprint interface ======================= */


static void await_finger_on_run_state(struct fpi_ssm *ssm){
  unsigned char * receive_buf;
  struct fp_img_dev *dev = (struct fp_img_dev*)ssm->priv;
  struct vfs7552_data *data = (struct vfs7552_data*)dev->priv;
  fp_dbg("state %d", ssm->cur_state);

  switch (ssm->cur_state){
    case AWAIT_FINGER_ON_INIT:
      data->init_sequence.stepcount =
  			array_n_elements(vfs7552_initiate_capture);
  		data->init_sequence.actions = vfs7552_initiate_capture;
  		data->init_sequence.timeout = 1000;
  		usb_exchange_async(ssm, &data->init_sequence);
      break;
    case AWAIT_FINGER_ON_INTERRUPT_QUERY:
      data->init_sequence.stepcount =
        array_n_elements(vfs7552_wait_finger_init);
      data->init_sequence.actions = vfs7552_wait_finger_init;
      data->init_sequence.timeout = 0; // Do not time out
      usb_exchange_async(ssm, &data->init_sequence);
      break;
    case AWAIT_FINGER_ON_INTERRUPT_CHECK:
      receive_buf = ((unsigned char *)data->init_sequence.receive_buf);
      if(receive_buf[0] == interrupt_ok[0]){
        // This seems to mean: "Sensor is all good"
        fpi_ssm_jump_to_state(ssm, AWAIT_FINGER_ON_INTERRUPT_QUERY);
      } else if(receive_buf[0] == interrupt_ready[0]){
        // This seems to mean: "We detected a finger"
        fpi_ssm_next_state(ssm);
      } else if(receive_buf[0] == interrupt_dont_ask[0]){
        // THis seems to mean: "We already told you we detected a finger, stop asking us"
        // It will not respond to an other request on the interrupt endpoint
        fpi_ssm_next_state(ssm);
      } else {
        fp_dbg("Unknown response 0x%02x", receive_buf[0]);
        fpi_ssm_next_state(ssm);
      }
      break;
  }
}

static void capture_run_state(struct fpi_ssm *ssm){
  struct fp_img_dev *dev = (struct fp_img_dev*)ssm->priv;
  struct vfs7552_data *data = (struct vfs7552_data*)dev->priv;
  unsigned char * receive_buf;
  int r;
  fp_dbg("state %d", ssm->cur_state);

  switch (ssm->cur_state){
  case CAPTURE_QUERY_DATA_READY:
    data->init_sequence.stepcount =
      array_n_elements(vfs7552_data_ready_query);
    data->init_sequence.actions = vfs7552_data_ready_query;
    data->init_sequence.timeout = 0; // Do not time out
    usb_exchange_async(ssm, &data->init_sequence);
    break;
  case CAPTURE_CHECK_DATA_READY:
    receive_buf = ((unsigned char *)data->init_sequence.receive_buf);
    if(receive_buf[0] == vfs7552_is_image_ready_resp_not_ready[0]){
      fpi_ssm_jump_to_state(ssm, CAPTURE_QUERY_DATA_READY);
    } else if(receive_buf[0] == vfs7552_is_image_ready_resp_ready[0]){
      capture_init(data);
      fpi_ssm_next_state(ssm);
    } else if(receive_buf[0] == vfs7552_is_image_ready_resp_finger_off[0]){
      fpi_ssm_jump_to_state(ssm, CAPTURE_DISABLE_SENSOR);
    } else {
      fp_dbg("Unknown response 0x%02x", receive_buf[0]);
      r = receive_buf[0];
      fpi_imgdev_session_error(dev, r);
      fpi_ssm_mark_aborted(ssm, r);
    }
    break;
  case CAPTURE_REQUEST_CHUNK:
    fp_dbg("Requesting chunk");
    data->init_sequence.stepcount =
      array_n_elements(vfs7552_request_chunk);
    data->init_sequence.actions = vfs7552_request_chunk;
    data->init_sequence.timeout = 1000;
    usb_exchange_async(ssm, &data->init_sequence);
    break;
  case CAPTURE_READ_CHUNK:
    r = capture_chunk_async(data, dev->udev,
          1000, ssm);
    if (r != 0) {
      fp_err("Failed to capture data");
      fpi_imgdev_session_error(dev, r);
      fpi_ssm_mark_aborted(ssm, r);
    }
    break;
  case CAPTURE_COMPLETE:
    if(data->activate_state == IMGDEV_STATE_CAPTURE){
      fpi_ssm_mark_completed(ssm);
    } else if(data->activate_state == IMGDEV_STATE_AWAIT_FINGER_OFF){
      int mean = 0;
      int variance = 0;

      for(int i = 0; i < VFS7552_IMAGE_SIZE; i++)
        mean = mean + data->image[i];
      mean = mean / VFS7552_IMAGE_SIZE;

      for(int i = 0; i < VFS7552_IMAGE_SIZE; i++)
        variance = variance + (data->image[i] - mean) * (data->image[i] - mean);
      variance = variance / (VFS7552_IMAGE_SIZE - 1);

      fp_dbg("mean = %d, variance = %d\n", mean, variance);

      if(variance < 20)
        fpi_ssm_jump_to_state(ssm, CAPTURE_DISABLE_SENSOR);
      else
        fpi_ssm_jump_to_state(ssm, CAPTURE_QUERY_DATA_READY);
    }
    break;

    case CAPTURE_DISABLE_SENSOR:
      data->init_sequence.stepcount =
        array_n_elements(vfs7552_stop_capture);
      data->init_sequence.actions = vfs7552_stop_capture;
      data->init_sequence.timeout = 1000;
      usb_exchange_async(ssm, &data->init_sequence);
      break;
    case CAPTURE_DISABLE_COMPLETE:
      fpi_ssm_mark_completed(ssm);
      break;
  }
}
static int execute_state_change(struct fp_img_dev *dev){
	struct vfs7552_data *data = (struct vfs7552_data *)dev->priv;
  struct fpi_ssm *ssm;

  fp_dbg("state %d", data->activate_state);
  switch (data->activate_state) {
  case IMGDEV_STATE_INACTIVE:
    // This state is never used...
    break;
  case IMGDEV_STATE_AWAIT_FINGER_ON:
    ssm = fpi_ssm_new(dev->dev, await_finger_on_run_state, AWAIT_FINGER_ON_NUM_STATES);
    ssm->priv = dev;
		fpi_ssm_start(ssm, report_finger_on); // await_finger_on_complete
		break;
  case IMGDEV_STATE_CAPTURE:
    ssm = fpi_ssm_new(dev->dev, capture_run_state, CAPTURE_NUM_STATES);
    ssm->priv = dev;
    fpi_ssm_start(ssm, submit_image); // await_finger_on_complete
    break;
  case IMGDEV_STATE_AWAIT_FINGER_OFF:
    ssm = fpi_ssm_new(dev->dev, capture_run_state, CAPTURE_NUM_STATES);
    ssm->priv = dev;
    fpi_ssm_start(ssm, report_finger_off); // await_finger_on_complete
    break;
  }
  return 0;
}

static void open_loop(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = (struct fp_img_dev *)ssm->priv;
	struct vfs7552_data *data = (struct vfs7552_data *)dev->priv;

	switch (ssm->cur_state) {
	case DEV_OPEN_START:
		data->init_sequence.stepcount =
			array_n_elements(vfs7552_initialization);
		data->init_sequence.actions = vfs7552_initialization;
		data->init_sequence.timeout = VFS7552_DEFAULT_WAIT_TIMEOUT;
		usb_exchange_async(ssm, &data->init_sequence);
		break;
	};
}

static void open_loop_complete(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = (struct fp_img_dev *)ssm->priv;

	fpi_imgdev_open_complete(dev, 0);
	fpi_ssm_free(ssm);
}

static int dev_open(struct fp_img_dev *dev, unsigned long driver_data)
{

	struct vfs7552_data *data;
	int r;

	data = (struct vfs7552_data *)g_malloc0(sizeof(*data));
  data->init_sequence.device = dev;
  data->init_sequence.receive_buf =
    g_malloc0(VFS7552_RECEIVE_BUF_SIZE);
	data->capture_buffer =
		(unsigned char *)g_malloc0(VFS7552_RECEIVE_BUF_SIZE);
  data->image =
    (unsigned char*)g_malloc0(VFS7552_IMAGE_HEIGHT * VFS7552_IMAGE_WIDTH);

	dev->priv = data;

	r = libusb_reset_device(dev->udev);
	if (r != 0) {
		fp_err("Failed to reset the device");
		return r;
	}

	r = libusb_claim_interface(dev->udev, 0);
	if (r != 0) {
		fp_err("Failed to claim interface: %s", libusb_error_name(r));
		return r;
	}

	struct fpi_ssm *ssm;
	ssm = fpi_ssm_new(dev->dev, open_loop, DEV_OPEN_NUM_STATES);
	ssm->priv = dev;
	fpi_ssm_start(ssm, open_loop_complete);

	return 0;
}

static void dev_close(struct fp_img_dev *dev)
{
	libusb_release_interface(dev->udev, 0);
	struct vfs7552_data *data = (struct vfs7552_data *)dev->priv;
	if (data != NULL) {
    if(data->init_sequence.receive_buf != NULL)
      g_free(data->init_sequence.receive_buf);
    g_free(data->capture_buffer);
    g_free(data->image);
		g_free(data);
	}
	fpi_imgdev_close_complete(dev);
}


static int dev_change_state(struct fp_img_dev *dev, enum fp_imgdev_state state)
{
	struct vfs7552_data *data = dev->priv;
	switch (state) {
	case IMGDEV_STATE_INACTIVE:
	case IMGDEV_STATE_AWAIT_FINGER_ON:
	case IMGDEV_STATE_CAPTURE:
  case IMGDEV_STATE_AWAIT_FINGER_OFF:
		break;
	default:
		fp_err("unrecognised state %d", state);
		return -EINVAL;
	}

	data->activate_state = state;

	return execute_state_change(dev);
}

// According to Igor Filatov, the parameter is not so useful:
// Not really. The same state is sent to dev_change_state after activation has ended with
// fpi_imgdev_activate_complete. You can ignore it.
// ALl the commands are sent in open...
// Maybe we should move open to activate?
static int dev_activate(struct fp_img_dev *dev, enum fp_imgdev_state state)
{
	struct vfs7552_data *data = (struct vfs7552_data *)dev->priv;

	fp_dbg("device initialized");
	data->deactivating = FALSE;

  fpi_imgdev_activate_complete(dev, 0);
	return 0;
}

static void dev_deactivate(struct fp_img_dev *dev)
{

  dev_change_state(dev, IMGDEV_STATE_INACTIVE);

	int r;
	struct vfs7552_data *data = dev->priv;

	if (data->loop_running) {
		data->deactivating = TRUE;
		if (data->flying_transfer) {
			r = libusb_cancel_transfer(data->flying_transfer);
			if (r < 0)
				fp_dbg("cancel failed error %d", r);
		}
	} else{
    fpi_imgdev_deactivate_complete(dev);
  }

}

static const struct usb_id id_table[] = {
	{ .vendor = 0x138a, .product = 0x0091 /* Validity device from some Dell XPS laptops (9560, 9360 at least) */ },
	{ 0, 0, 0, },
};

struct fp_img_driver vfs7552_driver = {
	.driver = {
		.id = VFS7552_ID,
		.name = "vfs7552",
		.full_name = "Validity VFS7552",
		.id_table = id_table,
		.scan_type = FP_SCAN_TYPE_PRESS,
	},

	.flags = 0, // What is this?
	.img_width = VFS7552_IMAGE_WIDTH,
	.img_height = VFS7552_IMAGE_HEIGHT,

	.open = dev_open,
	.close = dev_close,
	.activate = dev_activate,
	.deactivate = dev_deactivate,
  .change_state = dev_change_state,
};
