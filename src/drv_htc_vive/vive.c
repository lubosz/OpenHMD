// Copyright 2013, Fredrik Hultin.
// Copyright 2013, Jakob Bornecrantz.
// Copyright 2013, Joey Ferwerda.
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* HTC Vive Driver */


#define FEATURE_BUFFER_SIZE 256

#define HTC_ID                   0x0bb4
#define VIVE_HMD                 0x2c87
#define VIVE_PRO_HMD             0x0309

#define VALVE_ID                 0x28de
#define VIVE_WATCHMAN_DONGLE     0x2101
#define VIVE_LIGHTHOUSE_FPGA_RX  0x2000
#define VIVE_LHR                 0x2300 // VIVE PRO

#define VIVE_CLOCK_FREQ 48000000.0f // Hz = 48 MHz

#include <string.h>
#include <wchar.h>
#include <hidapi.h>
#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <stdbool.h>
#include <asm/byteorder.h>
#include <inttypes.h>

#include "vive.h"

typedef enum {
	REV_VIVE,
	REV_VIVE_PRO
} vive_revision;

typedef enum
{
	VIVE_HEADSET      = 1,
	VIVE_CONTROLLER_0 = 2,
	VIVE_CONTROLLER_1 = 3
} vive_device_type;

typedef struct {
	ohmd_device base;

	hid_device* imu_handle;
	vive_device_type type;
	vive_imu_config imu_config;
	fusion sensor_fusion;
	uint32_t last_ticks;

	// headset only
	hid_device* hmd_handle;
	vive_revision revision;
	vec3f raw_accel, raw_gyro;
	uint8_t last_seq;

	vec3f gyro_error;
	filter_queue gyro_q;

	// controller only
	uint8_t buttons;
	uint32_t last_controller_ticks2;

} vive_priv;



void vec3f_from_vive_vec_accel(const vive_imu_config* config,
                               const int16_t* smp,
                               vec3f* out)
{
	float range = config->acc_range / 32768.0f;
	out->x = range * config->acc_scale.x * (float) smp[0] - config->acc_bias.x;
	out->y = range * config->acc_scale.y * (float) smp[1] - config->acc_bias.y;
	out->z = range * config->acc_scale.z * (float) smp[2] - config->acc_bias.z;
}

void vec3f_from_vive_vec_gyro(const vive_imu_config* config,
                              const int16_t* smp,
                              vec3f* out)
{
	float range = config->gyro_range / 32768.0f;
	out->x = range * config->gyro_scale.x * (float)smp[0] - config->gyro_bias.x;
	out->y = range * config->gyro_scale.y * (float)smp[1] - config->gyro_bias.x;
	out->z = range * config->gyro_scale.z * (float)smp[2] - config->gyro_bias.x;
}

static bool process_error(vive_priv* priv)
{
	if(priv->gyro_q.at >= priv->gyro_q.size - 1)
		return true;

	ofq_add(&priv->gyro_q, &priv->raw_gyro);

	if(priv->gyro_q.at >= priv->gyro_q.size - 1){
		ofq_get_mean(&priv->gyro_q, &priv->gyro_error);
		LOGE("gyro error: %f, %f, %f\n",
		     priv->gyro_error.x, priv->gyro_error.y, priv->gyro_error.z);
	}

	return false;
}

vive_headset_imu_sample* get_next_sample(vive_headset_imu_packet* pkt,
                                         int last_seq)
{
	int diff[3];

	for(int i = 0; i < 3; i++)
	{
		diff[i] = (int)pkt->samples[i].seq - last_seq;

		if(diff[i] < -128){
			diff[i] += 256;
		}
	}

	int closest_diff = INT_MAX;
	int closest_idx = -1;

	for(int i = 0; i < 3; i++)
	{
		if(diff[i] < closest_diff && diff[i] > 0 && diff[i] < 128){
			closest_diff = diff[i];
			closest_idx = i;
		}
	}

	if(closest_idx != -1)
		return pkt->samples + closest_idx;

	return NULL;
}

static void handle_imu_packet(vive_priv* priv, unsigned char *buffer, int size)
{
	vive_headset_imu_packet pkt;
	vive_decode_sensor_packet(&pkt, buffer, size);

	vive_headset_imu_sample* smp = NULL;

	while((smp = get_next_sample(&pkt, priv->last_seq)) != NULL)
	{
		if(priv->last_ticks == 0)
			priv->last_ticks = smp->time_ticks;

		uint32_t t1, t2;
		t1 = smp->time_ticks;
		t2 = priv->last_ticks;

		float dt = (t1 - t2) / VIVE_CLOCK_FREQ;

		priv->last_ticks = smp->time_ticks;

		vec3f_from_vive_vec_accel(&priv->imu_config, smp->acc, &priv->raw_accel);
		vec3f_from_vive_vec_gyro(&priv->imu_config, smp->rot, &priv->raw_gyro);

		// Fix imu orientation
		switch (priv->revision) {
			case REV_VIVE:
				priv->raw_accel.y *= -1;
				priv->raw_accel.z *= -1;
				priv->raw_gyro.y *= -1;
				priv->raw_gyro.z *= -1;
				break;
			case REV_VIVE_PRO:
				priv->raw_accel.x *= -1;
				priv->raw_accel.z *= -1;
				priv->raw_gyro.x *= -1;
				priv->raw_gyro.z *= -1;
				break;
			default:
				LOGE("Unknown VIVE revision.\n");
		}

		if(process_error(priv)){
			vec3f mag = {{0.0f, 0.0f, 0.0f}};
			vec3f gyro;
			ovec3f_subtract(&priv->raw_gyro, &priv->gyro_error, &gyro);

			ofusion_update(&priv->sensor_fusion, dt,
			               &gyro, &priv->raw_accel, &mag);
		}

		priv->last_seq = smp->seq;
	}
}

static void read_headset_reports(vive_priv* priv)
{
	int size = 0;

	unsigned char buffer[FEATURE_BUFFER_SIZE];

	while((size = hid_read(priv->imu_handle, buffer, FEATURE_BUFFER_SIZE)) > 0) {
		if(buffer[0] == VIVE_HMD_IMU_PACKET_ID){
			handle_imu_packet(priv, buffer, size);
		}else{
			LOGE("unknown message type: %u", buffer[0]);
		}
	}

	if(size < 0){
		LOGE("error reading from device");
	}

}

static void controller_handle_battery(vive_priv* priv, uint8_t battery)
{
	uint8_t charge_percent = battery & VIVE_CONTROLLER_BATTERY_CHARGE_MASK;
	bool charging = battery & VIVE_CONTROLLER_BATTERY_CHARGING;

	//if (battery != self->battery)
	//	self->battery = battery;

	(void)charge_percent;
	(void)charging;
}

typedef struct  {
	uint32_t bit;
	uint8_t code;
} button_map;


static const button_map controller_button_map[6] = {
	{ VIVE_CONTROLLER_BUTTON_MENU, OHMD_MENU },
	{ VIVE_CONTROLLER_BUTTON_GRIP, OHMD_SQUEEZE },
	{ VIVE_CONTROLLER_BUTTON_SYSTEM, OHMD_HOME },
	{ VIVE_CONTROLLER_BUTTON_THUMB, OHMD_ANALOG_PRESS },
	{ VIVE_CONTROLLER_BUTTON_TOUCH, OHMD_TOUCHPAD_TOUCH },
	{ VIVE_CONTROLLER_BUTTON_TRIGGER, OHMD_TRIGGER_CLICK },
};

const char* vive_button_to_string(int button)
{
	switch (button)
	{
	case VIVE_CONTROLLER_BUTTON_MENU:
		return "VIVE_CONTROLLER_BUTTON_MENU";
	case VIVE_CONTROLLER_BUTTON_GRIP:
		return "VIVE_CONTROLLER_BUTTON_GRIP";
	case VIVE_CONTROLLER_BUTTON_SYSTEM:
		return "VIVE_CONTROLLER_BUTTON_SYSTEM";
	case VIVE_CONTROLLER_BUTTON_THUMB:
		return "VIVE_CONTROLLER_BUTTON_THUMB";
	case VIVE_CONTROLLER_BUTTON_TOUCH:
		return "VIVE_CONTROLLER_BUTTON_TOUCH";
	case VIVE_CONTROLLER_BUTTON_TRIGGER:
		return "VIVE_CONTROLLER_BUTTON_TRIGGER";
	}
}

#define BUTTON_PRESSED	0x80

void handle_buttons(uint32_t buttons, uint32_t last_buttons, uint8_t map_length,
                    const button_map *map)
{
	uint8_t btn_codes[map_length];
	int num_buttons = 0;
	int i;

	for (i = 0; i < map_length; i++) {
		uint32_t bit = map[i].bit;

		if ((buttons ^ last_buttons) & bit) {
			btn_codes[num_buttons] = map[i].code;
			if (buttons & bit) {
				printf("Pressed  %s\n", vive_button_to_string(bit));
				btn_codes[num_buttons] |= BUTTON_PRESSED;
			} else {
				printf("Released %s\n", vive_button_to_string(bit));
			}
			num_buttons++;
		}
	}

	//telemetry_send_buttons(dev_id, btn_codes, num_buttons);
}


static void controller_handle_buttons(vive_priv* priv, uint8_t buttons)
{

	if (buttons != priv->buttons) {
		handle_buttons(buttons, priv->buttons, 6, controller_button_map);
		priv->buttons = buttons;
	}

}

static void controller_handle_touch_position(vive_priv* priv, uint8_t *buf)
{
	int16_t x = __le16_to_cpup((__le16 *)buf);
	int16_t y = __le16_to_cpup((__le16 *)(buf + 2));

	//printf("Touch position: %d %d\n", x, y);
	/*
	if (x != self->touch_pos[0] ||
	    y != self->touch_pos[1]) {
		self->touch_pos[0] = x;
		self->touch_pos[1] = y;
	}
	*/
}

static void controller_handle_analog_trigger(vive_priv* priv,
                                             uint8_t squeeze)
{
	//printf("Analog trigger %d\n", squeeze);
	/*
	if (squeeze != self->squeeze)
		self->squeeze = squeeze;

	*/
}

static int controller_haptic_pulse(hid_device* device)
{
	const vive_controller_haptic_pulse_report report = {
		.id = VIVE_CONTROLLER_COMMAND_PACKET_ID,
		.command = VIVE_CONTROLLER_HAPTIC_PULSE_COMMAND,
		.len = 7,
		.unknown = { 0x00, 0xf4, 0x01, 0xb5, 0xa2, 0x01, 0x00 },
	};

	return hid_send_feature_report(device, &report, sizeof(report));
}

static int controller_poweroff(hid_device* device)
{
	const vive_controller_poweroff_report report = {
		.id = VIVE_CONTROLLER_COMMAND_PACKET_ID,
		.command = VIVE_CONTROLLER_POWEROFF_COMMAND,
		.len = 4,
		.magic = { 'o', 'f', 'f', '!' },
	};

	return hid_send_feature_report(device, &report, sizeof(report));
}

static void controller_handle_imu_sample(vive_priv* priv, uint8_t *buf)
{
	/* Time in 48 MHz ticks, but we are missing the low byte */
	uint32_t timestamp = priv->last_ticks | *buf;
	int16_t accel[3] = {
		__le16_to_cpup((__le16 *)(buf + 1)),
		__le16_to_cpup((__le16 *)(buf + 3)),
		__le16_to_cpup((__le16 *)(buf + 5)),
	};
	int16_t gyro[3] = {
		__le16_to_cpup((__le16 *)(buf + 7)),
		__le16_to_cpup((__le16 *)(buf + 9)),
		__le16_to_cpup((__le16 *)(buf + 11)),
	};

	if(priv->last_controller_ticks2 == 0)
		priv->last_controller_ticks2 = timestamp;
/*
	if (timestamp % 100 == 0)
	{
		printf("VIBRATE!\n");
		controller_haptic_pulse(priv);
	}
*/
	uint32_t t1, t2;
	t1 = timestamp;
	t2 = priv->last_controller_ticks2;

	float dt = (t1 - t2) / VIVE_CLOCK_FREQ;

	priv->last_controller_ticks2 = timestamp;

	float range = priv->imu_config.acc_range / 32768.0f;

	vec3f accel_vec;
	accel_vec.x = range * priv->imu_config.acc_scale.x * (float) accel[0] - priv->imu_config.acc_bias.x;
	accel_vec.y = range * priv->imu_config.acc_scale.y * (float) accel[1] - priv->imu_config.acc_bias.y;
	accel_vec.z = range * priv->imu_config.acc_scale.z * (float) accel[2] - priv->imu_config.acc_bias.z;

	float gyro_range = priv->imu_config.gyro_range / 32768.0f;
	vec3f gyro_vec;
	gyro_vec.x = gyro_range * priv->imu_config.gyro_scale.x * (float)gyro[0] - priv->imu_config.gyro_bias.x;
	gyro_vec.y = gyro_range * priv->imu_config.gyro_scale.y * (float)gyro[1] - priv->imu_config.gyro_bias.x;
	gyro_vec.z = gyro_range * priv->imu_config.gyro_scale.z * (float)gyro[2] - priv->imu_config.gyro_bias.x;

	vec3f mag = {{0.0f, 0.0f, 0.0f}};

	ofusion_update(&priv->sensor_fusion, dt,
	               &gyro_vec, &accel_vec, &mag);
}

/*
 * Decodes multiplexed Wireless Receiver messages.
 */
static void decode_controller_message(vive_priv* priv,
                                      vive_controller_message *message)
{
	unsigned char *buf = message->payload;
	unsigned char *end = message->payload + message->len - 1;
	bool silent = true;
	int i;

	//self->timestamp = (message->timestamp_hi << 24) |
	//		  (message->timestamp_lo << 16);

	priv->last_ticks = (message->timestamp_hi << 24) |
	                   (message->timestamp_lo << 16);
	// printf("timestamp: %ld\n", timestamp);

	/*
	 * Handle button, touch, and IMU events. The first byte of each event
	 * has the three most significant bits set.
	 */
	while ((buf < end) && ((*buf >> 5) == 7)) {
		uint8_t type = *buf++;

		if (type & 0x10) {
			if (type & 1) {
				controller_handle_buttons(priv, *buf++);
			}
			if (type & 2) {
				controller_handle_touch_position(priv, buf);
				buf += 4;
			}
			if (type & 4) {
				controller_handle_analog_trigger(priv, *buf++);
			}
		} else {
			if (type & 1)
				controller_handle_battery(priv, *buf++);
			if (type & 2) {
				/* unknown, does ever happen? */
				silent = false;
				buf++;
			}
		}
		if (type & 8) {
			controller_handle_imu_sample(priv, buf);
			buf += 13;
		}
	}

	if (buf > end)
		printf("overshoot: %ld\n", buf - end);
	//if (!silent || buf > end)
	//	vive_controller_dump_message(priv, message);
	if (buf >= end)
		return;

}

static void read_controller_reports(vive_priv* priv)
{
	int size = 0;

	unsigned char buffer[FEATURE_BUFFER_SIZE];

	while((size = hid_read(priv->imu_handle, buffer, FEATURE_BUFFER_SIZE)) > 0) {
		if(buffer[0] == VIVE_HMD_IMU_PACKET_ID){
			printf("got VIVE_HMD_IMU_PACKET_ID\n");
			//handle_imu_packet(priv, buffer, size);
		} else if (buffer[0] == VIVE_CONTROLLER_PACKET1_ID) {
			vive_controller_packet1 *pkt = (vive_controller_packet1 *) buffer;
			decode_controller_message(priv, &pkt->message);

		} else if (buffer[0] == VIVE_CONTROLLER_PACKET2_ID) {
			//LOGI("Got controller packet 2.");
		} else if (buffer[0] == VIVE_CONTROLLER_DISCONNECT_PACKET_ID) {
			LOGI("Got controller disconnected.");
		}else{
			LOGE("unknown message type: %u", buffer[0]);
		}
	}

	if(size < 0){
		LOGE("error reading from device");
	}
}

static void update_device(ohmd_device* device)
{
	vive_priv* priv = (vive_priv*)device;

	switch (priv->type)
	{
	case VIVE_HEADSET:
		read_headset_reports (priv);
		break;
	case VIVE_CONTROLLER_0:
		case VIVE_CONTROLLER_1:
		read_controller_reports(priv);
		break;
	}

}

static int getf(ohmd_device* device, ohmd_float_value type, float* out)
{
	vive_priv* priv = (vive_priv*)device;

	switch(type){
	case OHMD_ROTATION_QUAT:
		*(quatf*)out = priv->sensor_fusion.orient;
		break;

	case OHMD_POSITION_VECTOR:
		out[0] = out[1] = out[2] = 0;
		break;

	case OHMD_DISTORTION_K:
		// TODO this should be set to the equivalent of no distortion
		memset(out, 0, sizeof(float) * 6);
		break;

	default:
		ohmd_set_error(priv->base.ctx, "invalid type given to getf (%ud)", type);
		return -1;
		break;
	}

	return 0;
}

static void close_device(ohmd_device* device)
{
	int hret = 0;
	vive_priv* priv = (vive_priv*)device;

	LOGD("closing HTC Vive device");

	// turn the display off
	switch (priv->revision) {
		case REV_VIVE:
			hret = hid_send_feature_report(priv->hmd_handle,
			                               vive_magic_power_off1,
			                               sizeof(vive_magic_power_off1));
			LOGI("power off magic 1: %d\n", hret);

			hret = hid_send_feature_report(priv->hmd_handle,
			                               vive_magic_power_off2,
			                               sizeof(vive_magic_power_off2));
			LOGI("power off magic 2: %d\n", hret);
			break;
		case REV_VIVE_PRO:
			hret = hid_send_feature_report(priv->hmd_handle,
			                               vive_pro_magic_power_off,
			                               sizeof(vive_pro_magic_power_off));
			LOGI("vive pro power off magic: %d\n", hret);
			break;
		default:
			LOGE("Unknown VIVE revision.\n");
	}

	switch (priv->type)
	{
		case VIVE_HEADSET:
			hid_close(priv->hmd_handle);
			break;
		case VIVE_CONTROLLER_0:
		case VIVE_CONTROLLER_1:
			controller_poweroff(priv->imu_handle);
			break;
	}

	hid_close(priv->imu_handle);

	free(device);
}

#if 0
static void dump_indexed_string(hid_device* device, int index)
{
	wchar_t wbuffer[512] = {0};
	char buffer[1024] = {0};

	int hret = hid_get_indexed_string(device, index, wbuffer, 511);

	if(hret == 0){
		wcstombs(buffer, wbuffer, sizeof(buffer));
		LOGD("indexed string 0x%02x: '%s'\n", index, buffer);
	}
}
#endif

static void dump_info_string(int (*fun)(hid_device*, wchar_t*, size_t),
                            const char* what, hid_device* device)
{
	wchar_t wbuffer[512] = {0};
	char buffer[1024] = {0};

	int hret = fun(device, wbuffer, 511);

	if(hret == 0){
		wcstombs(buffer, wbuffer, sizeof(buffer));
		LOGI("%s: '%s'\n", what, buffer);
	}
}

#if 0
static void dumpbin(const char* label, const unsigned char* data, int length)
{
	printf("%s:\n", label);
	for(int i = 0; i < length; i++){
		printf("%02x ", data[i]);
		if((i % 16) == 15)
			printf("\n");
	}
	printf("\n");
}
#endif

static hid_device* open_device_idx(int manufacturer, int product, int iface,
                                   int iface_tot, int device_index)
{
	struct hid_device_info* devs = hid_enumerate(manufacturer, product);
	struct hid_device_info* cur_dev = devs;

	int idx = 0;
	int iface_cur = 0;
	hid_device* ret = NULL;

	while (cur_dev) {
		LOGI("%04x:%04x %s\n", manufacturer, product, cur_dev->path);

		if(idx == device_index && iface == iface_cur){
			ret = hid_open_path(cur_dev->path);
			LOGI("opening\n");
		}

		cur_dev = cur_dev->next;

		iface_cur++;

		if(iface_cur >= iface_tot){
			idx++;
			iface_cur = 0;
		}
	}

	hid_free_enumeration(devs);

	return ret;
}

int vive_read_firmware(hid_device* device)
{
	vive_firmware_version_packet packet = {
		.id = VIVE_FIRMWARE_VERSION_PACKET_ID,
	};

	int bytes;

	LOGI("Getting vive_firmware_version_packet...");
	bytes = hid_get_feature_report(device,
	                               (unsigned char*) &packet,
	                               sizeof(packet));

	if (bytes < 0)
	{
		LOGE("Could not get vive_firmware_version_packet: %d", bytes);
		return bytes;
	}

	LOGI("Firmware version %u %s@%s FPGA %u.%u\n",
		packet.firmware_version, packet.string1,
		packet.string2, packet.fpga_version_major,
		packet.fpga_version_minor);
	LOGI("Hardware revision: %d rev %d.%d.%d\n",
		packet.hardware_revision, packet.hardware_version_major,
		packet.hardware_version_minor, packet.hardware_version_micro);

	return 0;
}

int vive_read_config(vive_priv* priv)
{
	vive_config_start_packet start_packet = {
		.id = VIVE_CONFIG_START_PACKET_ID,
	};

	int bytes;

	LOGI("Getting vive_config_start_packet...");
	bytes = hid_get_feature_report(priv->imu_handle,
	                               (unsigned char*) &start_packet,
	                               sizeof(start_packet));

	if (bytes < 0)
	{
		LOGE("Could not get vive_config_start_packet: %d", bytes);
		return bytes;
	}

	LOGI("Config packet size is %i bytes.", bytes);

	vive_config_read_packet read_packet = {
		.id = VIVE_CONFIG_READ_PACKET_ID,
	};

	unsigned char* packet_buffer = malloc(4096);

	int offset = 0;
	do {
		bytes = hid_get_feature_report(priv->imu_handle,
		                               (unsigned char*) &read_packet,
		                               sizeof(read_packet));

		memcpy((uint8_t*)packet_buffer + offset,
		       &read_packet.payload,
		       read_packet.length);
		offset += read_packet.length;
	} while (read_packet.length);
	packet_buffer[offset] = '\0';
	vive_decode_config_packet(&priv->imu_config, packet_buffer, offset);

	free(packet_buffer);

	return 0;
}

#define OHMD_GRAVITY_EARTH 9.80665 // m/s²

int vive_get_range_packet(vive_priv* priv)
{
	int ret;

	vive_imu_range_modes_packet packet = {
		.id = VIVE_IMU_RANGE_MODES_PACKET_ID
	};

	ret = hid_get_feature_report(priv->imu_handle,
	                             (unsigned char*) &packet,
	                             sizeof(packet));

	if (ret < 0)
	{
		LOGE("Could not get feature report %d.", packet.id);
		return ret;
	}

	if (!packet.gyro_range || !packet.accel_range)
	{
		LOGW("Invalid gyroscope and accelerometer data. Trying to fetch again.");
		ret = hid_get_feature_report(priv->imu_handle,
		                             (unsigned char*) &packet,
		                             sizeof(packet));
		if (ret < 0)
		{
			LOGE("Could not get feature report %d.", packet.id);
			return ret;
		}

		if (!packet.gyro_range || !packet.accel_range)
		{
			LOGE("Unexpected range mode report: %02x %02x %02x",
				packet.id, packet.gyro_range, packet.accel_range);
			for (int i = 0; i < 61; i++)
				printf(" %02x", packet.unknown[i]);
			printf("\n");
			return -1;
		}
	}

	if (packet.gyro_range > 4 || packet.accel_range > 4)
	{
		LOGE("Gyroscope or accelerometer range too large.");
		return -1;
	}

	/*
	 * Convert MPU-6500 gyro full scale range (+/-250°/s, +/-500°/s,
	 * +/-1000°/s, or +/-2000°/s) into rad/s, accel full scale range
	 * (+/-2g, +/-4g, +/-8g, or +/-16g) into m/s².
	 */

	double gyro_range = M_PI / 180.0 * (250 << packet.gyro_range);
	priv->imu_config.gyro_range = (float) gyro_range;
	LOGI("Vive gyroscope range     %f", gyro_range);

	double acc_range = OHMD_GRAVITY_EARTH * (2 << packet.accel_range);
	priv->imu_config.acc_range = (float) acc_range;
	LOGI("Vive accelerometer range %f", acc_range);
	return 0;
}

static int open_controller(vive_priv* priv, int idx, uint32_t i)
{
	printf("Opening controller!\n");
	priv->imu_handle = open_device_idx(VALVE_ID,
	                                   VIVE_WATCHMAN_DONGLE, i, 2, idx);

	if(!priv->imu_handle) {
    LOGE("Could not open watchman dongle %d!", i);
    return -1;
	}

  if(hid_set_nonblocking(priv->imu_handle, 1) == -1){
		LOGE("Failed to set non-blocking on device");
    return -1;
	}

	if (vive_read_firmware(priv->imu_handle) != 0)
	{
		LOGE("Could not get watchman firmware version!");
	}

	//vive_read_config_controller(priv);

  if (vive_read_config(priv) != 0)
	{
		LOGE("Could not get watchman config!");
	}

  if (vive_get_range_packet(priv) != 0)
	{
		LOGW("Could not get controller imu range packet.\n");
	}
  return 0;
}

static int open_headset(vive_priv* priv, int idx)
{
	int hret;

	// Open the HMD device
	switch (priv->revision) {
		case REV_VIVE:
			priv->hmd_handle = open_device_idx(HTC_ID, VIVE_HMD, 0, 1, idx);
			break;
		case REV_VIVE_PRO:
			priv->hmd_handle = open_device_idx(HTC_ID, VIVE_PRO_HMD, 0, 1, idx);
			break;
		default:
			LOGE("Unknown VIVE revision.\n");
			return -1;
	}

	if(!priv->hmd_handle)
		return -1;

	if(hid_set_nonblocking(priv->hmd_handle, 1) == -1){
		ohmd_set_error(priv->base.ctx, "failed to set non-blocking on device");
		return -1;
	}

	switch (priv->revision) {
		case REV_VIVE:
			priv->imu_handle = open_device_idx(VALVE_ID,
			                                   VIVE_LIGHTHOUSE_FPGA_RX, 0, 2, idx);
			break;
		case REV_VIVE_PRO:
			priv->imu_handle = open_device_idx(VALVE_ID, VIVE_LHR, 0, 1, idx);
			break;
		default:
			LOGE("Unknown VIVE revision.\n");
			return -1;
	}

	if(!priv->imu_handle)
		return -1;

	if(hid_set_nonblocking(priv->imu_handle, 1) == -1){
		ohmd_set_error(priv->base.ctx, "failed to set non-blocking on device");
		return -1;
	}

	dump_info_string(hid_get_manufacturer_string,
	                 "manufacturer", priv->hmd_handle);
	dump_info_string(hid_get_product_string, "product", priv->hmd_handle);
	dump_info_string(hid_get_serial_number_string,
	                 "serial number", priv->hmd_handle);

#if 0
	// enable lighthouse
	hret = hid_send_feature_report(priv->hmd_handle,
	                               vive_magic_enable_lighthouse,
	                               sizeof(vive_magic_enable_lighthouse));
	LOGD("enable lighthouse magic: %d\n", hret);
#endif

	switch (priv->revision) {
		case REV_VIVE:
			if (vive_read_config(priv) != 0)
			{
				LOGW("Could not read config. Using defaults.\n");
			}

			if (vive_get_range_packet(priv) != 0)
			{
				LOGW("Could not get range packet.\n");
			}

			if (vive_read_firmware(priv->imu_handle) != 0)
			{
				LOGE("Could not get headset firmware version!");
			}

			// turn the display on
			hret = hid_send_feature_report(priv->hmd_handle,
			                               vive_magic_power_on,
			                               sizeof(vive_magic_power_on));
			LOGI("power on magic: %d\n", hret);

			break;
		case REV_VIVE_PRO:
			// turn the display on
			hret = hid_send_feature_report(priv->hmd_handle,
			                               vive_pro_magic_power_on,
			                               sizeof(vive_pro_magic_power_on));
			LOGI("power on magic: %d\n", hret);

			// Enable VIVE Pro IMU
			hret = hid_send_feature_report(priv->imu_handle,
			                               vive_pro_enable_imu,
			                               sizeof(vive_pro_enable_imu));
			LOGI("Enable Pro IMU magic: %d\n", hret);
			break;
		default:
			LOGE("Unknown VIVE revision.\n");
	}

	// Set default device properties
	ohmd_set_default_device_properties(&priv->base.properties);

	// Set device properties TODO: Get from device
	switch (priv->revision) {
		case REV_VIVE:
			priv->base.properties.hres = 2160;
			priv->base.properties.vres = 1200;
			priv->base.properties.ratio = (2160.0f / 1200.0f) / 2.0f;
			break;
		case REV_VIVE_PRO:
			priv->base.properties.hres = 2880;
			priv->base.properties.vres = 1600;
			priv->base.properties.ratio = (2880.0f / 1600.0f) / 2.0f;
			break;
		default:
			LOGE("Unknown VIVE revision.\n");
	}

	//TODO: Confirm exact mesurements. Get for VIVE Pro.
	priv->base.properties.hsize = 0.122822f;
	priv->base.properties.vsize = 0.068234f;
	priv->base.properties.lens_sep = 0.063500f;
	priv->base.properties.lens_vpos = 0.049694f;
	priv->base.properties.fov = DEG_TO_RAD(111.435f);

	// calculate projection eye projection matrices from the device properties
	ohmd_calc_default_proj_matrices(&priv->base.properties);

	return 0;
}

static ohmd_device* open_device(ohmd_driver* driver, ohmd_device_desc* desc)
{
	vive_priv* priv = ohmd_alloc(driver->ctx, sizeof(vive_priv));

	printf("🥨 Opening device %d\n\n", desc->id);

	if(!priv)
		return NULL;

	priv->base.ctx = driver->ctx;
	priv->revision = desc->revision;
	priv->type = desc->id;

	int idx = atoi(desc->path);

	/* IMU config defaults */
	priv->imu_config.acc_bias.x = 0;
	priv->imu_config.acc_bias.y = 0;
	priv->imu_config.acc_bias.z = 0;

	priv->imu_config.acc_scale.x = 1.0f;
	priv->imu_config.acc_scale.y = 1.0f;
	priv->imu_config.acc_scale.z = 1.0f;

	priv->imu_config.gyro_bias.x = 0;
	priv->imu_config.gyro_bias.y = 0;
	priv->imu_config.gyro_bias.z = 0;

	priv->imu_config.gyro_scale.x = 1.0f;
	priv->imu_config.gyro_scale.y = 1.0f;
	priv->imu_config.gyro_scale.z = 1.0f;

	priv->imu_config.gyro_range = 8.726646f;
	priv->imu_config.acc_range = 39.226600f;

	switch (priv->type)
	{
	case VIVE_HEADSET:
		if (open_headset(priv, idx) < 0)
			goto cleanup;
		break;
	case VIVE_CONTROLLER_0:
		if (open_controller(priv, idx, 0) < 0)
		  goto cleanup;
		break;
	case VIVE_CONTROLLER_1:
		if (open_controller(priv, idx, 1) < 0)
		  goto cleanup;
		break;
	}

	// set up device callbacks
	priv->base.update = update_device;
	priv->base.close = close_device;
	priv->base.getf = getf;

	ofusion_init(&priv->sensor_fusion);

	ofq_init(&priv->gyro_q, 128);

	return (ohmd_device*)priv;

cleanup:
	if(priv)
		free(priv);

	return NULL;
}

static void get_device_list(ohmd_driver* driver, ohmd_device_list* list)
{
	vive_revision rev;
	struct hid_device_info* devs = hid_enumerate(HTC_ID, VIVE_HMD);

	if (devs != NULL) {
		rev = REV_VIVE;
	} else {
		devs = hid_enumerate(HTC_ID, VIVE_PRO_HMD);
		if (devs != NULL)
			rev = REV_VIVE_PRO;
	}

	struct hid_device_info* cur_dev = devs;

	int idx = 0;
	while (cur_dev) {
		ohmd_device_desc* desc = &list->devices[list->num_devices++];

		strcpy(desc->driver, "OpenHMD HTC Vive Driver");
		strcpy(desc->vendor, "HTC/Valve");
		strcpy(desc->product, "HTC Vive");

		desc->revision = rev;

		snprintf(desc->path, OHMD_STR_SIZE, "%d", idx);

		desc->driver_ptr = driver;
		desc->device_class = OHMD_DEVICE_CLASS_HMD;
		desc->device_flags = OHMD_DEVICE_FLAGS_ROTATIONAL_TRACKING;

		desc->id = VIVE_HEADSET;

		// Controller 0
		desc = &list->devices[list->num_devices++];

		strcpy(desc->driver, "OpenHMD HTC Vive Driver");
		strcpy(desc->vendor, "HTC/Valve");
		strcpy(desc->product, "HTC Vive: Controller 0");

		strcpy(desc->path, cur_dev->path);

		desc->device_flags = OHMD_DEVICE_FLAGS_ROTATIONAL_TRACKING;
		desc->device_class = OHMD_DEVICE_CLASS_CONTROLLER;

		desc->driver_ptr = driver;
		desc->id = VIVE_CONTROLLER_0;

		// Controller 1
		desc = &list->devices[list->num_devices++];

		strcpy(desc->driver, "OpenHMD HTC Vive Driver");
		strcpy(desc->vendor, "HTC/Valve");
		strcpy(desc->product, "HTC Vive: Controller 1");

		strcpy(desc->path, cur_dev->path);

		desc->device_flags = OHMD_DEVICE_FLAGS_ROTATIONAL_TRACKING;
		desc->device_class = OHMD_DEVICE_CLASS_CONTROLLER;

		desc->driver_ptr = driver;
		desc->id = VIVE_CONTROLLER_1;

		cur_dev = cur_dev->next;
	}

	hid_free_enumeration(devs);
}

static void destroy_driver(ohmd_driver* drv)
{
	LOGD("shutting down HTC Vive driver");
	free(drv);
}

ohmd_driver* ohmd_create_htc_vive_drv(ohmd_context* ctx)
{
	ohmd_driver* drv = ohmd_alloc(ctx, sizeof(ohmd_driver));

	if(!drv)
		return NULL;

	drv->get_device_list = get_device_list;
	drv->open_device = open_device;
	drv->destroy = destroy_driver;
	drv->ctx = ctx;

	return drv;
}
