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
 *           3       1       366
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
 *     m_loop { A, 1, m_read, C, 3, m_next }
 */

enum {
	M_LOOP_A,
	M_LOOP_1,
	M_LOOP_READ,
	M_LOOP_C,
	M_LOOP_3,
	M_LOOP_NEXT,
	M_LOOP_NUM_STATES,
};

struct validity_dev {
	int seqnum;
};

#define EP_IN(n)			(n | LIBUSB_ENDPOINT_IN)
#define EP_OUT(n)			(n | LIBUSB_ENDPOINT_OUT)

#define BULK_TIMEOUT 20

#define PKTSIZE 292


/* The first two bytes of data will be overwritten with seqnum */
static int send(struct fp_img_dev *dev, int n, unsigned char *data, size_t len)
{
	struct validity_dev *vdev = dev->priv;
	int transferred;
	int r;

	fp_dbg("seq:%04x len:%zd", vdev->seqnum, len);

	data[0] = vdev->seqnum & 0xff;
	data[1] = (vdev->seqnum>>8) & 0xff;

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

	if (transferred < len) {
		fp_err("unexpected short read %d/%zd", transferred, len);
		return -EIO;
	} else {
		return 0;
	}
}

static void do_q(struct fp_img_dev *dev)
{
	unsigned char q1[0x07] = { 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00 };
	unsigned char q2[0x0a] = { 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x55, 0x00, 0x08, 0x00 };
	unsigned char rr[0x30];
	send (dev, 1, q1, 0x07);
	recv (dev, 1, rr, 0x30);
	send (dev, 1, q1, 0x07);
	recv (dev, 1, rr, 0x30);
	send (dev, 1, q2, 0x0a);
	recv (dev, 1, rr, 0x0a);
}

static void do_b(struct fp_img_dev *dev)
{
	unsigned char b1[0x08] = { 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x14, 0x00 };
	unsigned char b2[0x06] = { 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00 };
	unsigned char b3[0x08] = { 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x11, 0x00 };
	unsigned char b4[0x0a] = { 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x62, 0x00, 0x32, 0x00 };
	unsigned char rr[0x40000];
	send (dev, 1, b1, 0x08);
	recv (dev, 1, rr, 0x0a);
	send (dev, 1, b2, 0x06);
	recv (dev, 1, rr, 0x08);
	recv (dev, 2, rr, 0x40000);	// flush hw output buffer?
	send (dev, 1, b3, 0x08);
	recv (dev, 1, rr, 0x0a);	// this comes back with one bit different on linux...
	send (dev, 1, b4, 0x0a);
	recv (dev, 1, rr, 0x0a);
}

static void do_d(struct fp_img_dev *dev)
{
	unsigned char b1[0x08] = { 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x14, 0x00 };
	unsigned char rr[0x0a];
	send (dev, 1, b1, 0x08);
	recv (dev, 1, rr, 0x0a);
}

static void do_e(struct fp_img_dev *dev)
{
	unsigned char b1[0x0e] = { 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x88, 0x13, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01 };
	unsigned char rr[0x08];
	send (dev, 1, b1, 0x0e);
	recv (dev, 1, rr, 0x08);
}

static void do_2(struct fp_img_dev *dev)
{
	unsigned char b1[0x0e] = { 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x14, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01 };
	unsigned char rr[20*PKTSIZE];
	send (dev, 1, b1, 0x0e);
	recv (dev, 1, rr, 0x08);
	recv (dev, 2, rr, 20*PKTSIZE);	// read a small image?
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
		break;
	}
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
	fpi_ssm_free(ssm);
}

static int dev_activate(struct fp_img_dev *dev, enum fp_imgdev_state state)
{
	struct validity_dev *vdev = dev->priv;
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

