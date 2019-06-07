/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <usb/usb_device.h>
#include <usb/class/usb_hid.h>
#include <misc/printk.h>

static const u8_t hid_kbd_report_desc[] = HID_KEYBOARD_REPORT_DESC();
static bool usb_connected = false;

static void in_ready_cb(void)
{

}

static void status_cb(enum usb_dc_status_code status, const u8_t *param)
{
	switch (status) {
	case USB_DC_CONFIGURED:
		usb_connected = true;
		break;
	case USB_DC_SUSPEND:
		usb_connected = false;
		break;
	default:
		break;
	}
}

static const struct hid_ops ops = {
	.int_in_ready = in_ready_cb,
	.status_cb = status_cb,
};

void main(void)
{
	struct device *hid;
	uint32_t count = 1;
	uint8_t rep[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	printk("stm32 usb sem example(%s)\n", CONFIG_BOARD);

	hid = device_get_binding("HID_0");
	if (hid == NULL) {
		printf("Cannot get USB HID 0 Device");
		return;
	}

	usb_hid_register_device(hid, hid_kbd_report_desc, sizeof(hid_kbd_report_desc), &ops);
	usb_hid_init(hid);

	
	rep[7] = HID_KEY_7;
	while (1) {
		if (usb_connected == true) {
			// "12345678901234567890"
			rep[7] = HID_KEY_1;
			hid_int_ep_write(hid, rep, sizeof(rep), NULL);

			rep[7] = HID_KEY_2;
			hid_int_ep_write(hid, rep, sizeof(rep), NULL);

			rep[7] = HID_KEY_3;
			hid_int_ep_write(hid, rep, sizeof(rep), NULL);

			rep[7] = HID_KEY_4;
			hid_int_ep_write(hid, rep, sizeof(rep), NULL);

			rep[7] = HID_KEY_5;
			hid_int_ep_write(hid, rep, sizeof(rep), NULL);

			rep[7] = HID_KEY_6;
			hid_int_ep_write(hid, rep, sizeof(rep), NULL);

			rep[7] = HID_KEY_7;
			hid_int_ep_write(hid, rep, sizeof(rep), NULL);

			rep[7] = HID_KEY_8;
			hid_int_ep_write(hid, rep, sizeof(rep), NULL);

			rep[7] = HID_KEY_9;
			hid_int_ep_write(hid, rep, sizeof(rep), NULL);

			rep[7] = HID_KEY_0;
			hid_int_ep_write(hid, rep, sizeof(rep), NULL);

			rep[7] = HID_KEY_1;
			hid_int_ep_write(hid, rep, sizeof(rep), NULL);

			rep[7] = HID_KEY_2;
			hid_int_ep_write(hid, rep, sizeof(rep), NULL);

			rep[7] = HID_KEY_3;
			hid_int_ep_write(hid, rep, sizeof(rep), NULL);

			rep[7] = HID_KEY_4;
			hid_int_ep_write(hid, rep, sizeof(rep), NULL);

			rep[7] = HID_KEY_5;
			hid_int_ep_write(hid, rep, sizeof(rep), NULL);

			rep[7] = HID_KEY_6;
			hid_int_ep_write(hid, rep, sizeof(rep), NULL);

			rep[7] = HID_KEY_7;
			hid_int_ep_write(hid, rep, sizeof(rep), NULL);

			rep[7] = HID_KEY_8;
			hid_int_ep_write(hid, rep, sizeof(rep), NULL);

			rep[7] = HID_KEY_9;
			hid_int_ep_write(hid, rep, sizeof(rep), NULL);

			rep[7] = HID_KEY_0;
			hid_int_ep_write(hid, rep, sizeof(rep), NULL);

			rep[7] = 0x00;
			hid_int_ep_write(hid, rep, sizeof(rep), NULL);

			rep[7] = HID_KEY_ENTER;
			hid_int_ep_write(hid, rep, sizeof(rep), NULL);
		}

		printf("100ms repeat print(%d)\n", count);
		k_sleep(100);
		count++;
	}
}
