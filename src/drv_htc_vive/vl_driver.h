/*
 * OpenHMD
 *
 * Copyright (C) 2013 Fredrik Hultin
 * Copyright (C) 2013 Jakob Bornecrantz
 *
 * Vive Libre
 *
 * Copyright (C) 2016 Lubosz Sarnecki <lubosz.sarnecki@collabora.co.uk>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin Street, Suite 500,
 * Boston, MA 02110-1335, USA.
 */

#pragma once

#ifdef __cplusplus

#include <hidapi.h>
#include <opencv2/opencv.hpp>

#include "vl_magic.h"
#include "vl_fusion.h"
#include "vl_messages.h"
#include "omath.h"

#define FEATURE_BUFFER_SIZE 256

#define HTC_ID                   0x0bb4
#define VIVE_HMD                 0x2c87

#define VALVE_ID                 0x28de
#define VIVE_WATCHMAN_DONGLE     0x2101
#define VIVE_LIGHTHOUSE_FPGA_RX  0x2000
#define VIVE_CONTROLLER          0x2012

#define FREQ_48MHZ 1.0f / 48000000.0f


class vl_driver {

    std::mutex mutex_hmd_device;

    public:
    hid_device* hmd_device;
    hid_device* hmd_imu_device;
    hid_device* watchman_dongle_device;
    hid_device* hmd_light_sensor_device;
    uint32_t previous_ticks;
    vl_fusion *sensor_fusion;


    vl_driver();
    ~vl_driver();
    bool init_devices(unsigned index);
    bool open_devices(int idx);
    void update_pose();

    void _update_pose(const vive_headset_imu_report &pkt);
    
    void log_watchman(hid_device *dev);
    void log_hmd_light(hid_device *dev);
    void log_hmd_imu(hid_device* dev);
    
    void send_hmd_on();
    void send_hmd_off();
    void send_enable_lighthouse();

    std::map<uint32_t, cv::Point3f> get_config_positions();

    std::map<uint32_t, std::vector<uint32_t>>
    poll_angles(char channel, uint32_t samples);

    std::pair<std::vector<float>, std::vector<float> >
    poll_pnp(char channel, uint32_t samples);

		std::map<uint32_t, cv::Point3f> config_sensor_positions;
};

typedef std::function<void(unsigned char*,int)> query_fun;

static std::map<uint32_t, cv::Point3f> get_config_positions_dev(hid_device * dev);

#define FEATURE_BUFFER_SIZE 256
static inline void hid_query(hid_device* dev, query_fun fun) {
    int size = 0;
    unsigned char buffer[FEATURE_BUFFER_SIZE];

    while((size = hid_read(dev, buffer, FEATURE_BUFFER_SIZE)) > 0)
        fun(buffer, size);

    if(size < 0)
        printf("error reading from device\n");
}
#endif /* __cplusplus */

#ifdef __cplusplus
extern "C" {
#endif

extern vec3f get_position(hid_device* imu_dev, hid_device* light_dev, struct vive_headset_lighthouse_pulse2 *samples_collection, char channel);

#ifdef __cplusplus
}
#endif
