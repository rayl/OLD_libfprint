/*
 * Validity driver for libfprint
 * Copyright (C) 2009 Ray Lehtiniemi <rayl@mail.com>
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

#define FP_COMPONENT "validity"

#include <errno.h>
#include <string.h>
#include <unistd.h>

#include <glib.h>
#include <libusb.h>

#include <fp_internal.h>

/*
 * The Validity sensor seems to operate as follows, where:
 *    - s labels a recurring block of USB transfers
 *    - n is the number of transfers in the block
 *    - b is how many <=16 byte lines are transferred
 *  
 *           s       n        b
 *          ---     ---      ---
 *     init: Q       3        10
 *           B       4         8
 *           2       1       366
 *           D       1         2
 *           B       4         8
 *           E       1         2
 *     loop: A       n        2n    * 50 ms poll for finger
 *           1       1     20001
 *           B       4         8
 *           2       1       366
 *           C      10        24
 *           2       1       366
 *           D       2         2
 *           B       4         8
 *           E       1         2
 *           loop
 *  
 * This is modelled with the following state machines:
 *  
 *     m_init { Q, m_read, m_next }
 *     m_read { B, 2 }
 *     m_next { D, B, E }
 *     m_loop { A, 1, m_read, C, 2, m_next }
 *
 *
 * The data that comes back in blocks 1 and 2 above
 * seems to be split up into 292 byte packets. block 2 
 * contains exactly 20 packets, while block 1 contains a variable
 * number of packets.
 *
 * Each packet is structured as follows:
 *
 *   01 FE
 *   5E 00
 *    -- -- -- --  -- -- -- --  -- -- -- --  -- -- -- --
 *    -- -- -- --  -- -- -- --  -- -- -- --  -- -- -- --
 *    -- -- -- --  -- -- -- --  -- -- -- --  -- -- -- --
 *    -- -- -- --  -- -- -- --  -- -- -- --  -- -- -- --
 *    -- -- -- --  -- -- -- --  -- -- -- --  -- -- -- --
 *    -- -- -- --  -- -- -- --  -- -- -- --  -- -- -- --
 *    -- -- -- --  -- -- -- --  -- -- -- --  -- -- -- --
 *    -- -- -- --  -- -- -- --  -- -- -- --  -- -- -- --
 *    -- -- -- --  -- -- -- --  -- -- -- --  -- -- -- --
 *    -- -- -- --  -- -- -- --  -- -- -- --  -- -- -- --
 *    -- -- -- --  -- -- -- --  -- -- -- --  -- -- -- --
 *    -- -- -- --  -- -- -- --  -- -- -- --  -- -- -- --
 *    -- -- -- --  -- -- -- --  -- -- -- --  -- -- -- --
 *    -- -- -- --  -- -- -- --  -- -- -- --  -- -- -- --
 *    -- -- -- --  -- -- -- --  -- -- -- --  -- -- -- --
 *    -- -- -- --  -- -- -- --  -- -- -- --  -- -- -- --
 *    -- -- -- --  -- -- -- --  -- -- -- --  -- -- -- --
 *   14 03 6A 00
 *   00 5E
 *    -- -- -- --  -- -- -- --  -- -- -- --  -- --
 *
 * First two bytes are "01 FE" (except every 20th packet is "01 01")
 *
 * Next two bytes seem to be some kind of offset which increments
 * by about 0x1f or 0x20 on each packet (except every 20th packet
 * is a bit mangled)
 *
 * This is followed by 268 bytes of variable data.
 *
 * Then a constant "14 03 6A 00" header
 *
 * Then the index from bytes 2-3, but swapped.
 *
 * Then 14 bytes of variable data.
 */

struct validity_dev {
	int seqnum;
};


/******************************************************************************************************/
static inline int lo (int n)
{
	return n & 0xff;
}

static inline int hi (int n)
{
	return (n>>8) & 0xff;
}


/******************************************************************************************************/
#define EP_IN(n)			(n | LIBUSB_ENDPOINT_IN)
#define EP_OUT(n)			(n | LIBUSB_ENDPOINT_OUT)

#define BULK_TIMEOUT 20

#define PKTSIZE 292

/******************************************************************************************************/
/* The first two bytes of data will be overwritten with seqnum */
static int send(struct fp_img_dev *dev, int n, unsigned char *data, size_t len)
{
	struct validity_dev *vdev = dev->priv;
	int transferred;
	int r;

	fp_dbg("seq:%04x len:%zd", vdev->seqnum, len);

	data[0] = lo(vdev->seqnum);
	data[1] = hi(vdev->seqnum);

	r = libusb_bulk_transfer(dev->udev, EP_OUT(n), data, len, &transferred, BULK_TIMEOUT);

	if (r < 0) {
		fp_err("bulk write error %d", r);
		return r;

	} else if (transferred < len) {
		fp_err("unexpected short write %d/%zd", transferred, len);
		return -EIO;

	} else {
		return 0;
	}
}

static int recv(struct fp_img_dev *dev, int n, unsigned char *data, size_t len)
{
	struct validity_dev *vdev = dev->priv;
	int transferred;
	int r;

	fp_dbg("seq:%04x len:%zd", vdev->seqnum, len);

	r = libusb_bulk_transfer(dev->udev, EP_IN(n), data, len, &transferred, BULK_TIMEOUT);

	if (r < 0 && r != -7) {
		fp_err("bulk read error %d", r);
		return r;
	}

	vdev->seqnum++;

	return 0;
}

static int swap (struct fp_img_dev *dev, unsigned char *data, size_t len)
{
	unsigned char r[0x40];
	send(dev, 1, data, len);
	usleep(2000);
	recv(dev, 1, r, 0x40);
}

static int load (struct fp_img_dev *dev)
{
	unsigned char rr[0x40000];
	recv(dev, 2, rr, 0x40000);
}


/******************************************************************************************************
  Protocol notes from Damir Syabitov on vfs101driver group:

     00 00 01 00    - Reset
     00 00 02 00    - GetVersion
     00 00 03 00    - GetPrint
     00 00 04 00    - GetParam
     00 00 05 00    - SetParam
     00 00 06 00    - GetConfiguration
     00 00 07 00      DownloadPatch
     00 00 08 00      GetRateData
     00 00 09 00      IspRequest
     00 00 0A 00      ProgramFlash
     00 00 0B 00      EraseFlash
     00 00 0C 00      LedStates
     00 00 0D 00      LedEvent
     00 00 0E 00    - AbortPrint
     00 00 0F 00      Spare2
     00 00 10 00      Spare3
     00 00 11 00      Spare4
     00 00 12 00      Peek
     00 00 13 00      Poke
     00 00 14 00      SensorSpiTrans
     00 00 15 00      SensorGPIO
     00 00 16 00    - GetFingerState 
*/

/* Reset (00 00 01 00)
 *
 *  Cause the device to reenumerate on the USB bus.
 */
static void Reset (struct fp_img_dev *dev)
{
	unsigned char q1[0x07] = { 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00 };
	swap (dev, q1, 0x07);
}

/* GetVersion (00 00 01 00)
 *
 *  Retrieve version string from the device.
 */
static void GetVersion (struct fp_img_dev *dev)
{
	unsigned char q1[0x07] = { 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00 };
	swap (dev, q1, 0x07);
}

/* GetPrint (00 00 03 00)
 *
 *  Retrieve fingerprint image information.
 */
static void GetPrint (struct fp_img_dev *dev)
{
}

/* GetParam (00 00 04 00)
 *
 *  Retrieve a parameter value from the device.
 */
static void GetParam (struct fp_img_dev *dev, int param)
{
	unsigned char q1[0x08] = { 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00 };
	q1[6] = lo(param);
	q1[7] = hi(param);
	swap (dev, q1, 0x08);
}

/* SetParam (00 00 05 00)
 *
 *  Set a parameter value on the device.
 */
static void SetParam (struct fp_img_dev *dev, int param, int value)
{
	unsigned char q1[0x0a] = { 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00 };
	q1[6] = lo(param);
	q1[7] = hi(param);
	q1[8] = lo(value);
	q1[9] = hi(value);
	swap (dev, q1, 0x0a);
}

/* GetConfiguration (00 00 06 00)
 *
 *  Retrieve config info from the device.
 */
static void GetConfiguration (struct fp_img_dev *dev)
{
	unsigned char q1[0x06] = { 0x00, 0x00, 0x00, 0x00, 0x06, 0x00 };
	swap (dev, q1, 0x06);
}

/* AbortPrint (00 00 0e 00)
 *
 *  Abort the current scan operation.
 */
static void AbortPrint (struct fp_img_dev *dev)
{
	unsigned char q1[0x06] = { 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00 };
	swap (dev, q1, 0x06);
}

/* GetFingerState (00 00 16 00)
 *
 *  Poll device for current finger state.
 */
static void GetFingerState (struct fp_img_dev *dev)
{
	unsigned char q1[0x06] = { 0x00, 0x00, 0x00, 0x00, 0x16, 0x00 };
	swap (dev, q1, 0x06);
}


/******************************************************************************************************/
static void do_q (struct fp_img_dev *dev)
{
	GetVersion (dev);
	SetParam (dev, 0x55, 0x08);
}

static void do_b (struct fp_img_dev *dev)
{
	GetParam (dev, 0x14);
	AbortPrint (dev);
	load (dev);		// flush hw output buffer?
	GetParam (dev, 0x11);	// this comes back different on linux...
				//    expect  xxxx0000 04000000 0800
				//    receive xxxx0000 04000000 0000
	SetParam (dev, 0x62, 0x32);
}

static void do_c (struct fp_img_dev *dev)
{
	GetConfiguration (dev);
	GetParam (dev, 0x2e);
	GetVersion (dev);
	AbortPrint (dev);
	SetParam (dev, 0x55, 0x08);
	SetParam (dev, 0x55, 0x08);
	SetParam (dev, 0x55, 0x08);
	GetParam (dev, 0x14);
	GetParam (dev, 0x11);
	SetParam (dev, 0x62, 0x32);
}

static void do_d (struct fp_img_dev *dev)
{
	GetParam (dev, 0x14);
}

static void do_e (struct fp_img_dev *dev)
{
	unsigned char b1[0x0e] = { 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x88, 0x13, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01 }; // GetPrint
	swap (dev, b1, 0x0e);	// this comes back different on linux...
				// expect  xxxx0000 03000000
				// receive xxxx0000 03000c00
}

static void do_1(struct fp_img_dev *dev)
{
	unsigned char q1[0x06] = { 0x00, 0x00, 0x00, 0x00, 0x16, 0x00 };
	// swap q1 until last byte of response is 0x02, just like finger detection, then load() large image
}

static void do_2(struct fp_img_dev *dev)
{
	unsigned char b1[0x0e] = { 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x14, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01 }; // GetPrint
	swap (dev, b1, 0x0e);
	load (dev);		// read a small image?
}


/******************************************************************************************************/
enum {
	M_NEXT_D,
	M_NEXT_B,
	M_NEXT_E,
	M_NEXT_NUM_STATES,
};

static void m_next_state(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = ssm->priv;

	switch (ssm->cur_state) {
	case M_NEXT_D:
		do_d(dev);
		fpi_ssm_next_state(ssm);
		break;

	case M_NEXT_B:
		do_b(dev);
		fpi_ssm_next_state(ssm);
		break;

	case M_NEXT_E:
		do_e(dev);
		fpi_ssm_next_state(ssm);
		break;
	}
}


/******************************************************************************************************/
enum {
	M_READ_B,
	M_READ_2,
	M_READ_NUM_STATES,
};

static void m_read_state(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = ssm->priv;

	switch (ssm->cur_state) {
	case M_READ_B:
		do_b(dev);
		fpi_ssm_next_state(ssm);
		break;

	case M_READ_2:
		do_2(dev);
		fpi_ssm_next_state(ssm);
		break;
	}
}


/******************************************************************************************************/
enum {
	M_LOOP_1,
	M_LOOP_READ,
	M_LOOP_C,
	M_LOOP_2,
	M_LOOP_NEXT,
	M_LOOP_NUM_STATES,
};

static void m_loop_state(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = ssm->priv;
	struct fpi_ssm *subsm;

	switch (ssm->cur_state) {
	case M_LOOP_1:
		fp_dbg("*****************************************************************");
		do_1(dev);
		fpi_ssm_next_state(ssm);
		break;

	case M_LOOP_READ:
		subsm = fpi_ssm_new(dev->dev, m_read_state, M_READ_NUM_STATES);
		subsm->priv = dev;
		fpi_ssm_start_subsm(ssm, subsm);
		break;

	case M_LOOP_C:
		do_c(dev);
		fpi_ssm_next_state(ssm);
		break;

	case M_LOOP_2:
		do_2(dev);
		fpi_ssm_next_state(ssm);
		break;

	case M_LOOP_NEXT:
		subsm = fpi_ssm_new(dev->dev, m_next_state, M_NEXT_NUM_STATES);
		subsm->priv = dev;
		fpi_ssm_start_subsm(ssm, subsm);
		break;
	}
}

static void m_loop_complete(struct fpi_ssm *ssm)
{
	fp_dbg("status %d", ssm->error);
	fpi_ssm_free(ssm);
}


/******************************************************************************************************/
static void finger_detection_cb(struct libusb_transfer *transfer);

static void start_finger_detection(struct fp_img_dev *dev)
{
	unsigned char q1[0x06] = { 0x00, 0x00, 0x00, 0x00, 0x16, 0x00 };
	unsigned char rr[0x0b];
	struct libusb_transfer *transfer;
	unsigned char *data;
	int r;

	do {
		usleep(50000);
		swap (dev, q1, 0x06);
	} while (rr[0x0a] != 0x02);

	return;

	transfer = libusb_alloc_transfer(0);
	data = g_malloc(0x0b);
	libusb_fill_bulk_transfer(transfer, dev->udev, EP_IN(1), data, 0x0b, finger_detection_cb, dev, 2000);
	r = libusb_submit_transfer(transfer);
	if (r < 0) {
		g_free(data);
		libusb_free_transfer(transfer);
		fpi_imgdev_session_error(dev, r);
	}
}

static void finger_detection_cb(struct libusb_transfer *transfer)
{
	struct fp_img_dev *dev = transfer->user_data;
	unsigned char *data = transfer->buffer;

	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		fpi_imgdev_session_error(dev, -EIO);
		goto out;
	} else if (transfer->length != transfer->actual_length) {
		fpi_imgdev_session_error(dev, -EPROTO);
		goto out;
	}

	/* determine finger presence */
	if (data[0x0a] == 0x02) {
		struct fpi_ssm *ssm;
		/* finger present, start capturing */
		fp_dbg("found");
		fpi_imgdev_report_finger_status(dev, TRUE);
		ssm = fpi_ssm_new(dev->dev, m_loop_state, M_LOOP_NUM_STATES);
		ssm->priv = dev;
		fpi_ssm_start(ssm, m_loop_complete);
	} else {
		fp_dbg("again");
		/* no finger, poll for a new histogram */
		start_finger_detection(dev);
	}
out:
	g_free(data);
	libusb_free_transfer(transfer);
}

/******************************************************************************************************/
enum {
	M_INIT_Q,
	M_INIT_READ,
	M_INIT_NEXT,
	M_INIT_NUM_STATES,
};

static void m_init_state(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = ssm->priv;
	struct fpi_ssm *subsm;

	switch (ssm->cur_state) {
	case M_INIT_Q:
		do_q(dev);
		fpi_ssm_next_state(ssm);
		break;

	case M_INIT_READ:
		subsm = fpi_ssm_new(dev->dev, m_read_state, M_READ_NUM_STATES);
		subsm->priv = dev;
		fpi_ssm_start_subsm(ssm, subsm);
		break;

	case M_INIT_NEXT:
		subsm = fpi_ssm_new(dev->dev, m_next_state, M_NEXT_NUM_STATES);
		subsm->priv = dev;
		fpi_ssm_start_subsm(ssm, subsm);
		break;
	}
}

static void m_init_complete(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = ssm->priv;
	fp_dbg("status %d", ssm->error);
	fpi_imgdev_activate_complete(dev, ssm->error);
	if (!ssm->error)
		start_finger_detection(dev);
	fpi_ssm_free(ssm);
}



/******************************************************************************************************/
static int dev_activate(struct fp_img_dev *dev, enum fp_imgdev_state state)
{
	struct fpi_ssm *ssm = fpi_ssm_new(dev->dev, m_init_state, M_INIT_NUM_STATES);
	ssm->priv = dev;
	fpi_ssm_start(ssm, m_init_complete);
	return 0;
}

static void dev_deactivate(struct fp_img_dev *dev)
{
	fpi_imgdev_deactivate_complete(dev);
}

static int dev_init(struct fp_img_dev *dev, unsigned long driver_data)
{
	int r;
	struct validity_dev *vdev = g_malloc0(sizeof(struct validity_dev));

	vdev->seqnum = 0;
	dev->priv = vdev;

	r = libusb_claim_interface(dev->udev, 0);
	if (r < 0)
		fp_err("could not claim interface 0");

	if (r == 0)
		fpi_imgdev_open_complete(dev, 0);

	return r;
}

static void dev_deinit(struct fp_img_dev *dev)
{
	g_free(dev->priv);
	libusb_release_interface(dev->udev, 0);
	fpi_imgdev_close_complete(dev);
}

static const struct usb_id id_table[] = {
	{ .vendor = 0x138a, .product = 0x0001 },
	{ 0, 0, 0, },
};

struct fp_img_driver validity_driver = {
	.driver = {
		.id = 10,
		.name = FP_COMPONENT,
		.full_name = "Validity",
		.id_table = id_table,
		.scan_type = FP_SCAN_TYPE_SWIPE,
	},
	.flags = 0,
	.img_height = -1,
	.img_width = 128,

	.open = dev_init,
	.close = dev_deinit,
	.activate = dev_activate,
	.deactivate = dev_deactivate,
};

