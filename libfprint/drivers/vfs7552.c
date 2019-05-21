/*
 * Validity Sensors, Inc. VFS5011 Fingerprint Reader driver for libfprint
 * Copyright (C) 2013 Arseniy Lartsev <arseniy@chalmers.se>
 *                    AceLan Kao <acelan.kao@canonical.com>
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

#include "drivers_api.h"
#include "vfs7552_proto.h"

#define array_n_elements(array) (sizeof(array) / sizeof(array[0]))

#define VFS7552_CONTROL_PIXELS (8)
#define VFS7552_LINE_SIZE (VFS7552_IMAGE_WIDTH + VFS7552_CONTROL_PIXELS)
#define VFS7552_IMAGE_CHUNKS (3)

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
	.data = COMMAND \
},

#define RECV(ENDPOINT, SIZE) \
{ \
	.type = ACTION_RECEIVE, \
	.endpoint = ENDPOINT, \
	.size = SIZE, \
	.data = NULL \
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

//static void start_scan(struct fp_img_dev *dev);

static void async_send_cb(struct libusb_transfer *transfer)
{
	fpi_ssm *ssm = transfer->user_data;
	struct usbexchange_data *data = fpi_ssm_get_user_data(ssm);
	struct usb_action *action;

	if (fpi_ssm_get_cur_state(ssm) >= data->stepcount) {
		fp_err("Radiation detected!");
		fpi_imgdev_session_error(data->device, -EINVAL);
		fpi_ssm_mark_failed(ssm, -EINVAL);
		goto out;
	}

	action = &data->actions[fpi_ssm_get_cur_state(ssm)];
	if (action->type != ACTION_SEND) {
		fp_err("Radiation detected!");
		fpi_imgdev_session_error(data->device, -EINVAL);
		fpi_ssm_mark_failed(ssm, -EINVAL);
		goto out;
	}

	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		/* Transfer not completed, return IO error */
		fp_err("transfer not completed, status = %d", transfer->status);
		fpi_imgdev_session_error(data->device, -EIO);
		fpi_ssm_mark_failed(ssm, -EIO);
		goto out;
	}
	if (transfer->length != transfer->actual_length) {
		/* Data sended mismatch with expected, return protocol error */
		fp_err("length mismatch, got %d, expected %d",
			transfer->actual_length, transfer->length);
		fpi_imgdev_session_error(data->device, -EIO);
		fpi_ssm_mark_failed(ssm, -EIO);
		goto out;
	}

	/* success */
	fpi_ssm_next_state(ssm);

out:
	libusb_free_transfer(transfer);
}

static void async_recv_cb(struct libusb_transfer *transfer)
{
	fpi_ssm *ssm = transfer->user_data;
	struct usbexchange_data *data = fpi_ssm_get_user_data(ssm);
	struct usb_action *action;

	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		/* Transfer not completed, return IO error */
		fp_err("transfer not completed, status = %d", transfer->status);
		fpi_imgdev_session_error(data->device, -EIO);
		fpi_ssm_mark_failed(ssm, -EIO);
		goto out;
	}

	if (fpi_ssm_get_cur_state(ssm) >= data->stepcount) {
		fp_err("Radiation detected!");
		fpi_imgdev_session_error(data->device, -EINVAL);
		fpi_ssm_mark_failed(ssm, -EINVAL);
		goto out;
	}

	action = &data->actions[fpi_ssm_get_cur_state(ssm)];
	if (action->type != ACTION_RECEIVE) {
		fp_err("Radiation detected!");
		fpi_imgdev_session_error(data->device, -EINVAL);
		fpi_ssm_mark_failed(ssm, -EINVAL);
		goto out;
	}

	if (action->data != NULL) {
		if (transfer->actual_length != action->correct_reply_size) {
			fp_err("Got %d bytes instead of %d",
				transfer->actual_length,
				action->correct_reply_size);
			fpi_imgdev_session_error(data->device, -EIO);
			fpi_ssm_mark_failed(ssm, -EIO);
			goto out;
		}
		if (memcmp(transfer->buffer, action->data,
					action->correct_reply_size) != 0) {
			fp_dbg("Wrong reply:");
			fpi_imgdev_session_error(data->device, -EIO);
			fpi_ssm_mark_failed(ssm, -EIO);
			goto out;
		}
	} else
		fp_dbg("Got %d bytes out of %d", transfer->actual_length,
		       transfer->length);

	fpi_ssm_next_state(ssm);
out:
	libusb_free_transfer(transfer);
}

static void usbexchange_loop(fpi_ssm *ssm, struct fp_dev *_dev, void *user_data)
{
	//fp_dbg("VALIDITY91: inside usbexchange_loop");
	struct usbexchange_data *data = user_data;
	if (fpi_ssm_get_cur_state(ssm) >= data->stepcount) {
		fp_err("Bug detected: state %d out of range, only %d steps",
				fpi_ssm_get_cur_state(ssm), data->stepcount);
		fpi_imgdev_session_error(data->device, -EINVAL);
		fpi_ssm_mark_failed(ssm, -EINVAL);
		return;
	}

	struct usb_action *action = &data->actions[fpi_ssm_get_cur_state(ssm)];
	struct libusb_transfer *transfer;
	int ret = -EINVAL;

	switch (action->type) {
	case ACTION_SEND:
		fp_dbg("VALIDITY91: Sending %s", action->name);
		transfer = fpi_usb_alloc();
		/*
		if (transfer == NULL) {
			fp_err("VALIDITY91: Failed to allocate transfer");
			fpi_imgdev_session_error(data->device, -ENOMEM);
			fpi_ssm_mark_failed(ssm, -ENOMEM);
			return;
		}
		*/
		libusb_fill_bulk_transfer(transfer, fpi_dev_get_usb_dev(FP_DEV(data->device)),
					  action->endpoint, action->data,
					  action->size, async_send_cb, ssm,
					  data->timeout);
		ret = libusb_submit_transfer(transfer);
		break;

	case ACTION_RECEIVE:
		fp_dbg("VALIDITY91: Receiving %d bytes", action->size);
		transfer = fpi_usb_alloc();
		/*
		if (transfer == NULL) {
			fp_err("VALIDITY91: Failed to allocate transfer");
			fpi_imgdev_session_error(data->device, -ENOMEM);
			fpi_ssm_mark_failed(ssm, -ENOMEM);
			return;
		}
		*/
		libusb_fill_bulk_transfer(transfer, fpi_dev_get_usb_dev(FP_DEV(data->device)),
					  action->endpoint, data->receive_buf,
					  action->size, async_recv_cb, ssm,
					  data->timeout);
		fp_dbg("VALIDITY91: bulk transfer filled");
		ret = libusb_submit_transfer(transfer);
		break;

	default:
		fp_err("VALIDITY91: Bug detected: invalid action %d", action->type);
		fpi_imgdev_session_error(data->device, -EINVAL);
		fpi_ssm_mark_failed(ssm, -EINVAL);
		return;
	}

	if (ret != 0) {
		fp_err("VALIDITY91: USB transfer error: %s", strerror(ret));
		fpi_imgdev_session_error(data->device, ret);
		fpi_ssm_mark_failed(ssm, ret);
	}
	fp_dbg("VALIDITY91: usb action successful");
}

static void usb_exchange_async(fpi_ssm *ssm,
			       struct usbexchange_data *data)
{
	fpi_ssm *subsm = fpi_ssm_new(FP_DEV(data->device),
				     usbexchange_loop,
				     data->stepcount,
				     data);
	fpi_ssm_start_subsm(ssm, subsm);
}

/* ====================== utils ======================= */

/* Calculade squared standand deviation of sum of two lines */
/*
static int vfs5011_get_deviation2(struct fpi_line_asmbl_ctx *ctx, GSList *row1, GSList *row2)
{
	unsigned char *buf1, *buf2;
	int res = 0, mean = 0, i;
	const int size = 64;

	buf1 = row1->data + 56;
	buf2 = row2->data + 168;

	for (i = 0; i < size; i++)
		mean += (int)buf1[i] + (int)buf2[i];

	mean /= size;

	for (i = 0; i < size; i++) {
		int dev = (int)buf1[i] + (int)buf2[i] - mean;
		res += dev*dev;
	}

	return res / size;
}

static unsigned char vfs5011_get_pixel(struct fpi_line_asmbl_ctx *ctx,
				   GSList *row,
				   unsigned x)
{
	unsigned char *data = row->data + 8;

	return data[x];
}
*/
/* ====================== main stuff ======================= */

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

/*
enum {
	CAPTURE_LINES = 256,
	MAXLINES = 2000,
	MAX_CAPTURE_LINES = 100000,
};

static struct fpi_line_asmbl_ctx assembling_ctx = {
	.line_width = VFS5011_IMAGE_WIDTH,
	.max_height = MAXLINES,
	.resolution = 10,
	.median_filter_size = 25,
	.max_search_offset = 30,
	.get_deviation = vfs5011_get_deviation2,
	.get_pixel = vfs5011_get_pixel,
};

struct vfs5011_data {
	unsigned char *total_buffer;
	unsigned char *capture_buffer;
	unsigned char *row_buffer;
	unsigned char *lastline;
	GSList *rows;
	int lines_captured, lines_recorded, empty_lines;
	int max_lines_captured, max_lines_recorded;
	int lines_total, lines_total_allocated;
	gboolean loop_running;
	gboolean deactivating;
	struct usbexchange_data init_sequence;
	struct libusb_transfer *flying_transfer;
};

enum {
	DEV_ACTIVATE_REQUEST_FPRINT,
	DEV_ACTIVATE_INIT_COMPLETE,
	DEV_ACTIVATE_READ_DATA,
	DEV_ACTIVATE_DATA_COMPLETE,
	DEV_ACTIVATE_PREPARE_NEXT_CAPTURE,
	DEV_ACTIVATE_NUM_STATES
};
*/

enum {
	DEV_OPEN_START,
	DEV_OPEN_NUM_STATES
};


static void capture_init(struct vfs7552_data *data)
{
	fp_dbg("capture_init");
	data->image_index = 0;
	data->chunks_captured = 0;
	//data->lastline = NULL;
	//data->lines_captured = 0;
	//data->lines_recorded = 0;
	//data->empty_lines = 0;
	//data->lines_total = 0;
	//data->lines_total_allocated = 0;
	//data->total_buffer = NULL;
	//data->max_lines_captured = max_captured;
	//data->max_lines_recorded = max_recorded;
}

enum {
  CHUNK_READ_FINISHED,
  CHUNK_READ_NEED_MORE
};

static int process_chunk(struct vfs7552_data *data, int transferred)
{
	/*
	enum {
		DEVIATION_THRESHOLD = 15*15,
		DIFFERENCE_THRESHOLD = 600,
		STOP_CHECK_LINES = 50
	};
	*/

	fp_dbg("process_chunk: got %d bytes", transferred);
	//int lines_captured = transferred/VFS5011_LINE_SIZE;

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

void report_finger_off(struct fpi_ssm *ssm, struct fp_dev *d,
				 void *user_data){
  struct fp_img_dev *dev = fpi_ssm_get_user_data(ssm);

  fpi_imgdev_report_finger_status(dev, FALSE);
  fpi_ssm_free(ssm);
}
void report_finger_on(struct fpi_ssm *ssm, struct fp_dev *d,
				 void *user_data){
  struct fp_img_dev *dev = fpi_ssm_get_user_data(ssm);

  fpi_imgdev_report_finger_status(dev, TRUE);
  fpi_ssm_free(ssm);
}

void report_deactivate(struct fpi_ssm *ssm, struct fp_dev *d,
				 void *user_data){
  struct fp_img_dev *dev = fpi_ssm_get_user_data(ssm);

  fpi_imgdev_deactivate_complete(dev);
  fpi_ssm_free(ssm);
}

static void
submit_image(fpi_ssm             *ssm,
			struct fp_dev *d,
			void *user_data)
{
	struct fp_img_dev *dev = fpi_ssm_get_user_data(ssm);
	struct vfs7552_data *data;

	data = FP_INSTANCE_DATA(FP_DEV(dev));

	struct fp_img *img;


	img = fpi_img_new_for_imgdev(dev);
	memcpy(img->data, data->image, VFS7552_IMAGE_SIZE);
	fp_dbg("Image captured, commiting");
	fpi_imgdev_image_captured(dev, img);
	// fpi_ssm_free(ssm);
}

static void chunk_capture_callback(struct libusb_transfer *transfer)
{
	fpi_ssm *ssm = (fpi_ssm *)transfer->user_data;
	struct fp_img_dev *dev = fpi_ssm_get_user_data(ssm);
	struct vfs7552_data *data;

	data = FP_INSTANCE_DATA(FP_DEV(dev));

	if ((transfer->status == LIBUSB_TRANSFER_COMPLETED) ||
	    (transfer->status == LIBUSB_TRANSFER_TIMED_OUT)) {
		if (process_chunk(data, transfer->actual_length) == CHUNK_READ_FINISHED)
      fpi_ssm_next_state(ssm);
		else
			fpi_ssm_jump_to_state(ssm, CAPTURE_REQUEST_CHUNK);
	} else {
		if (!data->deactivating) {
			fp_err("Failed to capture data");
			fpi_ssm_mark_failed(ssm, -1);
		} else {
			fpi_ssm_mark_completed(ssm);
		}
	}

	/*
	if ((transfer->status == LIBUSB_TRANSFER_COMPLETED) ||
	    (transfer->status == LIBUSB_TRANSFER_TIMED_OUT)) {

		if (transfer->actual_length > 0)
			fpi_imgdev_report_finger_status(dev, TRUE);

		if (process_chunk(data, transfer->actual_length))
			fpi_ssm_jump_to_state(ssm, DEV_ACTIVATE_DATA_COMPLETE);
		else
			fpi_ssm_jump_to_state(ssm, DEV_ACTIVATE_READ_DATA);
	} else {
		if (!data->deactivating) {
			fp_err("Failed to capture data");
			fpi_ssm_mark_failed(ssm, -1);
		} else {
			fpi_ssm_mark_completed(ssm);
		}
	}
	*/
	libusb_free_transfer(transfer);
	data->flying_transfer = NULL;
}

static int capture_chunk_async(struct vfs7552_data *data,
			       libusb_device_handle *handle,
			       int timeout, fpi_ssm *ssm)
{
	/*
	fp_dbg("capture_chunk_async: capture %d lines, already have %d",
		nline, data->lines_recorded);
	enum {
		DEVIATION_THRESHOLD = 15*15,
		DIFFERENCE_THRESHOLD = 600,
		STOP_CHECK_LINES = 50
	};
	*/

	data->flying_transfer = fpi_usb_alloc();
	libusb_fill_bulk_transfer(data->flying_transfer, handle, VFS7552_IN_ENDPOINT,
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

static void await_finger_on_run_state(struct fpi_ssm *ssm, struct fp_dev *d,
				 void *user_data){
  unsigned char * receive_buf;
  struct fp_img_dev *dev = fpi_ssm_get_user_data(ssm);
  struct vfs7552_data *data; // = (struct vfs7552_data*)dev->priv;
  data = FP_INSTANCE_DATA(FP_DEV(dev));
  fp_dbg("state %d", fpi_ssm_get_cur_state(ssm));

  switch (fpi_ssm_get_cur_state(ssm)){
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

static void capture_run_state(struct fpi_ssm *ssm, struct fp_dev *d,
				 void *user_data){
	struct fp_img_dev *dev = fpi_ssm_get_user_data(ssm);
	struct vfs7552_data *data; // = (struct vfs7552_data*)dev->priv;
	data = FP_INSTANCE_DATA(FP_DEV(dev));
	unsigned char * receive_buf;
	int r;
	fp_dbg("VALIDITY91: state %d", fpi_ssm_get_cur_state(ssm));

	switch (fpi_ssm_get_cur_state(ssm)){
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
		  fpi_ssm_mark_failed(ssm, r);
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
		r = capture_chunk_async(data, fpi_dev_get_usb_dev(FP_DEV(dev)),
		      1000, ssm);
		if (r != 0) {
		  fp_err("Failed to capture data");
		  fpi_imgdev_session_error(dev, r);
		  fpi_ssm_mark_failed(ssm, r);
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

static void open_loop(fpi_ssm *ssm, struct fp_dev *_dev, void *user_data)
{
	struct fp_img_dev *dev = user_data;
	struct vfs7552_data *data;

	data = FP_INSTANCE_DATA(_dev);

	switch (fpi_ssm_get_cur_state(ssm)) {
	case DEV_OPEN_START:
		data->init_sequence.stepcount =
			G_N_ELEMENTS(vfs7552_initialization);
		data->init_sequence.actions = vfs7552_initialization;
		data->init_sequence.device = dev;
		data->init_sequence.receive_buf =
			g_malloc0(VFS7552_RECEIVE_BUF_SIZE);
		data->init_sequence.timeout = VFS7552_DEFAULT_WAIT_TIMEOUT;
		usb_exchange_async(ssm, &data->init_sequence);
		break;
	};
}

static void open_loop_complete(fpi_ssm *ssm, struct fp_dev *_dev, void *user_data)
{
	struct fp_img_dev *dev = user_data;
	struct vfs7552_data *data;

	data = FP_INSTANCE_DATA(_dev);
	g_free(data->init_sequence.receive_buf);
	data->init_sequence.receive_buf = NULL;

	fpi_imgdev_open_complete(dev, 0);
	fpi_ssm_free(ssm);
}

static int dev_open(struct fp_img_dev *dev, unsigned long driver_data)
{

	struct vfs7552_data *data;
	int r;

	data = (struct vfs7552_data *)g_malloc0(sizeof(*data));
	data->capture_buffer =
		(unsigned char *)g_malloc0(VFS7552_RECEIVE_BUF_SIZE);
	data->image =
    	(unsigned char*)g_malloc0(VFS7552_IMAGE_HEIGHT * VFS7552_IMAGE_WIDTH);
	fp_dev_set_instance_data(FP_DEV(dev), data);

	r = libusb_reset_device(fpi_dev_get_usb_dev(FP_DEV(dev)));
	if (r != 0) {
		fp_err("Failed to reset the device");
		return r;
	}

	r = libusb_claim_interface(fpi_dev_get_usb_dev(FP_DEV(dev)), 0);
	if (r != 0) {
		fp_err("Failed to claim interface: %s", libusb_error_name(r));
		return r;
	}

	fpi_ssm *ssm;
	ssm = fpi_ssm_new(FP_DEV(dev), open_loop, DEV_OPEN_NUM_STATES, dev);
	fpi_ssm_start(ssm, open_loop_complete);

	return 0;
}

static int execute_state_change(struct fp_img_dev *dev){
	struct vfs7552_data *data;
	data = FP_INSTANCE_DATA(FP_DEV(dev));
  struct fpi_ssm *ssm;

  fp_dbg("VALIDITY91: state %d", data->activate_state);
  switch (data->activate_state) {
  case IMGDEV_STATE_INACTIVE:
    // This state is never used...
    break;
  case IMGDEV_STATE_AWAIT_FINGER_ON:
    ssm = fpi_ssm_new(FP_DEV(dev), await_finger_on_run_state, AWAIT_FINGER_ON_NUM_STATES, dev);
    //ssm->priv = dev;
		fpi_ssm_start(ssm, report_finger_on); // await_finger_on_complete
		break;
  case IMGDEV_STATE_CAPTURE:
    ssm = fpi_ssm_new(FP_DEV(dev), capture_run_state, CAPTURE_NUM_STATES, dev);
    //ssm->priv = dev;
    fpi_ssm_start(ssm, submit_image); // await_finger_on_complete
    break;
  case IMGDEV_STATE_AWAIT_FINGER_OFF:
    ssm = fpi_ssm_new(FP_DEV(dev), capture_run_state, CAPTURE_NUM_STATES, dev);
    //ssm->priv = dev;
    fpi_ssm_start(ssm, report_finger_off); // await_finger_on_complete
    break;
  }
  return 0;
}

static int dev_change_state(struct fp_img_dev *dev, enum fp_imgdev_state state)
{
	fp_dbg("VALIDITY91: changing state");
	struct vfs7552_data *data;
	data = FP_INSTANCE_DATA(FP_DEV(dev));
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


static void dev_close(struct fp_img_dev *dev)
{
	libusb_release_interface(fpi_dev_get_usb_dev(FP_DEV(dev)), 0);
	struct vfs7552_data *data;
	data = FP_INSTANCE_DATA(FP_DEV(dev));
	if (data != NULL) {
		if(data->init_sequence.receive_buf != NULL)
			g_free(data->init_sequence.receive_buf);
		g_free(data->capture_buffer);
		//g_slist_free_full(data->rows, g_free);
		g_free(data->image);
		g_free(data);
	}
	fpi_imgdev_close_complete(dev);
}

/*
static void start_scan(struct fp_img_dev *dev)
{
	struct vfs7552_data *data;
	fpi_ssm *ssm;

	data = FP_INSTANCE_DATA(FP_DEV(dev));
	data->loop_running = TRUE;
	fp_dbg("creating ssm");
	ssm = fpi_ssm_new(FP_DEV(dev), activate_loop, DEV_ACTIVATE_NUM_STATES, dev);
	fp_dbg("starting ssm");
	fpi_ssm_start(ssm, activate_loop_complete);
	fp_dbg("ssm done, getting out");
}
*/

static int dev_activate(struct fp_img_dev *dev, enum fp_imgdev_state state)
{
	struct vfs7552_data *data;

	data = FP_INSTANCE_DATA(FP_DEV(dev));

	fp_dbg("VALIDITY91: device initialized");
	data->deactivating = FALSE;

	fpi_imgdev_activate_complete(dev, 0);
	return 0;
}

static void dev_deactivate(struct fp_img_dev *dev)
{
	dev_change_state(dev, IMGDEV_STATE_INACTIVE);

	int r;
	struct vfs7552_data *data;

	data = FP_INSTANCE_DATA(FP_DEV(dev));
	if (data->loop_running) {
		data->deactivating = TRUE;
		if (data->flying_transfer) {
			r = libusb_cancel_transfer(data->flying_transfer);
			if (r < 0)
				fp_dbg("cancel failed error %d", r);
		}
	} else
		fpi_imgdev_deactivate_complete(dev);
}

static const struct usb_id id_table[] = {
	// Until you can reliably detect a fingerprint,
    // we should not ship this driver for any device.
    /* Validity device from some Dell XPS laptops (9560, 9360 at least) */
	{ .vendor = 0x138a, .product = 0x0091 }, 
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

	.flags = 0,
	.img_width = VFS7552_IMAGE_WIDTH,
	.img_height = VFS7552_IMAGE_HEIGHT,
	//.bz3_threshold = 20,

	.open = dev_open,
	.close = dev_close,
	.activate = dev_activate,
	.deactivate = dev_deactivate,
	.change_state = dev_change_state,
};

