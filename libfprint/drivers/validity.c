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

struct validity_dev {
	int foo;
};

static int dev_activate(struct fp_img_dev *dev, enum fp_imgdev_state state)
{
	fpi_imgdev_activate_complete(dev, 0);
	return 0;
}

static void dev_deactivate(struct fp_img_dev *dev)
{
	fpi_imgdev_deactivate_complete(dev);
}

static int dev_init(struct fp_img_dev *dev, unsigned long driver_data)
{
	int r;
	dev->priv = g_malloc0(sizeof(struct validity_dev));

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

