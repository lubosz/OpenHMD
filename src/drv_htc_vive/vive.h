// Copyright 2013, Fredrik Hultin.
// Copyright 2013, Jakob Bornecrantz.
// Copyright 2013, Joey Ferwerda.
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* HTC Vive Driver */


#ifndef VIVE_H
#define VIVE_H

#include <stdint.h>
#include <stdbool.h>

#include "../openhmdi.h"
#include "magic.h"

typedef enum
{
	VIVE_IMU_RANGE_MODES_PACKET_ID = 1,
	VIVE_CONFIG_START_PACKET_ID = 16,
	VIVE_CONFIG_READ_PACKET_ID = 17,
	VIVE_HMD_IMU_PACKET_ID = 32,
	VIVE_FIRMWARE_VERSION_PACKET_ID = 0x05,
  VIVE_CONTROLLER_PACKET1_ID = 0x23,
  VIVE_CONTROLLER_PACKET2_ID = 0x24,
  VIVE_CONTROLLER_DISCONNECT_PACKET_ID = 0x26,

} vive_irq_cmd;

#define VIVE_CONTROLLER_BATTERY_CHARGING		0x80
#define VIVE_CONTROLLER_BATTERY_CHARGE_MASK		0x7f

#define VIVE_CONTROLLER_BUTTON_TRIGGER			0x01
#define VIVE_CONTROLLER_BUTTON_TOUCH			0x02
#define VIVE_CONTROLLER_BUTTON_THUMB			0x04
#define VIVE_CONTROLLER_BUTTON_SYSTEM			0x08
#define VIVE_CONTROLLER_BUTTON_GRIP			0x10
#define VIVE_CONTROLLER_BUTTON_MENU			0x20

typedef struct
{
	int16_t acc[3];
	int16_t rot[3];
	uint32_t time_ticks;
	uint8_t seq;
} vive_headset_imu_sample;

typedef struct
{
	uint8_t report_id;
	vive_headset_imu_sample samples[3];
} vive_headset_imu_packet;

typedef struct
{
	uint8_t id;
	uint8_t unused[63];
} vive_config_start_packet;

typedef struct
{
	uint8_t id;
	uint8_t length;
	uint8_t payload[62];
} vive_config_read_packet;

typedef struct
{
	uint8_t id;
	uint8_t gyro_range;
	uint8_t accel_range;
	uint8_t unknown[61];
} vive_imu_range_modes_packet;

typedef struct
{
	vec3f acc_bias, acc_scale;
	float acc_range;
	vec3f gyro_bias, gyro_scale;
	float gyro_range;
} vive_imu_config;

typedef struct
{
	uint8_t id;
	uint32_t firmware_version;
	uint32_t unknown1;
	uint8_t string1[16];
	uint8_t string2[16];
	uint8_t hardware_version_micro;
	uint8_t hardware_version_minor;
	uint8_t hardware_version_major;
	uint8_t hardware_revision;
	uint32_t unknown2;
	uint8_t fpga_version_minor;
	uint8_t fpga_version_major;
	uint8_t reserved[13];
} __attribute__((packed)) vive_firmware_version_packet;

typedef struct
{
	uint8_t timestamp_hi;
	uint8_t len;
	uint8_t timestamp_lo;
	uint8_t payload[26];
} __attribute__((packed)) vive_controller_message;

typedef struct
{
	uint8_t id;
	vive_controller_message message;
} __attribute__((packed)) vive_controller_packet1;

typedef struct
{
	uint8_t id;
	vive_controller_message message[2];
} __attribute__((packed)) vive_controller_packet2;


void vec3f_from_vive_vec(const int16_t* smp, vec3f* out_vec);
bool vive_decode_sensor_packet(vive_headset_imu_packet* pkt,
                               const unsigned char* buffer,
                               int size);
bool vive_decode_config_packet(vive_imu_config* result,
                               const unsigned char* buffer,
                               uint16_t size);
bool vive_decode_controller_packet1(vive_controller_packet1* pkt,
                                    const unsigned char* buffer, int size);
#endif
