/*
 * Main loop and device enumeration
 * Copyright 2015 Philipp Zabel
 * Copyright 2017 Lubosz Sarnecki
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <linux/hidraw.h>

#include <libudev.h>

#define NUM_MATCHES	1

#define VID_HTC			0x0bb4
#define PID_VIVE		0x2c87

#define VID_VALVE		0x28de
#define PID_VIVE_HEADSET	0x2000

struct interface_match {
	int iface;
	const char *subsystem;
	const char *name;
};

struct device_match {
	uint16_t vid;
	uint16_t pid;
	const char *name;
	union {
		const char *subsystem;
		const struct interface_match *interfaces;
	};
	int num_interfaces;
};

static const struct device_match vive_match = {
	.vid = VID_VALVE,
	.pid = PID_VIVE_HEADSET,
	.name = "Vive Headset",
	.interfaces = (const struct interface_match[]){
	{ 0, .subsystem = "hidraw", .name = "IMU" },
	{ 1, .subsystem = "hidraw", .name = "Lighthouse RX" },
},
	.num_interfaces = 2,
};

struct vive_device {
	unsigned long id;
	union {
		char *devnode;
		char *devnodes[3];
	};
	char *name;
	char *serial;
	int active;
	union {
		int fd;
		int fds[3];
	};
	char *parent_devpath;
};

struct vive_device* vive_headset = NULL;


static inline int hid_send_feature_report(int fd, const void *data, size_t length)
{
	return ioctl(fd, HIDIOCSFEATURE(length), data);
}

static int lighthouse_start(struct vive_device *dev)
{
	int ret;
	int enable_sensors = 1;

	unsigned char buf[5] = { 0x04 };

	/* Enable vsync timestamps, enable/disable sensor reports */
	buf[1] = enable_sensors ? 0x00 : 0x01;
	ret = hid_send_feature_report(dev->fd, buf, sizeof(buf));
	if (ret < 0) {
		printf("First report returned null!\n", ret);
		return ret;
	}

	printf("First lighthouse magic ret: %d\n", ret);

	/*
	 * Reset Lighthouse Rx registers? Without this, inactive channels are
	 * not cleared to 0xff.
	 */
	buf[0] = 0x07;
	buf[1] = 0x02;
	ret = hid_send_feature_report(dev->fd, buf, sizeof(buf));
	printf("Second lighthouse magic ret: %d\n", ret);

	if (ret < 0) {
		printf("%s: Failed to enable Lighthouse Receiver\n",
						dev->name);
		return ret;
	}
	printf("Enabling lighthouse on %s. %d\n", dev->name, ret);

	return 0;
}

int ouvrt_device_start(struct vive_device *dev)
{
	int i, ret;

	printf("ouvrt_device_start: %s\n", dev->name);

	if (dev->active)
		return 0;

	for (i = 0; i < 3; i++) {
		if (dev->fds[i] == -1 && dev->devnodes[i] != NULL) {
			dev->fds[i] = open(dev->devnodes[i],
												 O_RDWR | O_NONBLOCK);
			if (dev->fds[i] == -1) {
				printf("%s: Failed to open '%s': %d\n",
								dev->name, dev->devnodes[i], errno);
				return -1;
			}
		}
	}

	ret = lighthouse_start(dev);
	if (ret < 0)
		return ret;

	return 0;
}

static void ouvrtd_device_add(struct udev_device *dev)
{
	const char *devnode, *parent_devpath, *serial, *subsystem, *value;
	uint16_t vid, pid;
	struct udev_device *parent;
	int i, j = 0, iface;
	
	subsystem = udev_device_get_subsystem(dev);
	if (!subsystem)
		return;

	parent = udev_device_get_parent_with_subsystem_devtype(dev,
																												 "usb", "usb_interface");
	if (!parent)
		return;

	value = udev_device_get_sysattr_value(parent, "bInterfaceNumber");
	if (!value)
		return;
	iface = atoi(value);

	parent = udev_device_get_parent(parent);
	if (!parent)
		return;

	parent_devpath = udev_device_get_devpath(parent);
	if (!parent_devpath)
		return;

	value = udev_device_get_sysattr_value(parent, "idVendor");
	if (!value)
		return;
	vid = strtol(value, NULL, 16);

	value = udev_device_get_sysattr_value(parent, "idProduct");
	if (!value)
		return;

	pid = strtol(value, NULL, 16);

	if (vid != vive_match.vid ||
			pid != vive_match.pid) {
		return;
	}

	for (j = 0; j < vive_match.num_interfaces; j++) {
		if (strcmp(vive_match.interfaces[j].subsystem,
							 subsystem) == 0 &&
				vive_match.interfaces[j].iface == iface) {
			break;
		}
	}

	if (j >= vive_match.num_interfaces) {
		printf("interfaces do not match.\n");
		return;
	}

	devnode = udev_device_get_devnode(dev);
	if (vive_match.num_interfaces)
		printf("udev: Found %s %s: %s\n", vive_match.name,
						vive_match.interfaces[j].name, devnode);
	else
		printf("udev: Found %s: %s\n", vive_match.name, devnode);

	/*
	 * If this is a new interface of an already existing multi-interface
	 * device, join the existing device.
	 */

	if (vive_headset != NULL) {
		if (vive_headset->devnodes[j]) {
			printf("udev: Interface %d occupied by %s\n",
							iface, vive_headset->devnodes[j]);
			return;
		} else {
			vive_headset->devnodes[j] = strdup(devnode);
		}

		for (j = 0; j < vive_match.num_interfaces; j++) {
			if (vive_headset->devnodes[j] == NULL)
				break;
		}
		if (j == vive_match.num_interfaces) {
			ouvrt_device_start(vive_headset);
		}

		return;
	}

	/* Otherwise create a new device */

	vive_headset = malloc (sizeof (struct vive_device));
	vive_headset->devnodes[0] = NULL;
	vive_headset->devnodes[1] = NULL;
	vive_headset->devnodes[2] = NULL;
	vive_headset->name = NULL;
	vive_headset->serial = NULL;
	vive_headset->active = 0;
	vive_headset->fds[0] = -1;
	vive_headset->fds[1] = -1;
	vive_headset->fds[2] = -1;

	vive_headset->parent_devpath = strdup(parent_devpath);
	if (!vive_headset->devnodes[j]) {
		if (vive_match.num_interfaces)
			vive_headset->devnodes[j] = strdup(devnode);
		else
			vive_headset->devnode = strdup(devnode);
	}
	if (vive_headset->name == NULL)
		vive_headset->name = strdup(vive_match.name);
}

static int ouvrtd_enumerate(struct udev *udev)
{
	struct udev_enumerate *enumerate;
	struct udev_list_entry *devices, *dev_list_entry;

	enumerate = udev_enumerate_new(udev);
	udev_enumerate_add_match_subsystem(enumerate, "hidraw");
	udev_enumerate_scan_devices(enumerate);
	devices = udev_enumerate_get_list_entry(enumerate);

	udev_list_entry_foreach(dev_list_entry, devices) {
		const char *path;
		struct udev_device *dev;

		path = udev_list_entry_get_name(dev_list_entry);
		dev = udev_device_new_from_syspath(udev, path);
		ouvrtd_device_add(dev);

		udev_device_unref(dev);
	}

	udev_enumerate_unref(enumerate);

	return 0;
}

int main(int argc, char *argv[])
{
	struct udev *udev;

	udev = udev_new();
	if (!udev)
		return -1;

	ouvrtd_enumerate(udev);

	udev_unref(udev);

	return 0;
}
