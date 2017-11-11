#include <time.h>
#include <sys/time.h>
#include <stdio.h>

#include <stdio.h>
#include <signal.h>
#include <string>
#include <map>

#include <json/value.h>
#include <json/reader.h>

#include "vl_driver.h"
#include "vl_config.h"
#include "vl_light.h"

#include "vl_python.h"

vl_driver* driver;


static void signal_interrupt_handler(int sig) {
    signal(sig, SIG_IGN);
    delete(driver);
    exit(0);
}

ViveLibre::ViveLibre() {
  connect();
}

ViveLibre::~ViveLibre() {
    delete(driver);
}

void ViveLibre::connect() {
    driver = new vl_driver();
    if (!driver->init_devices(0))
        return;
    signal(SIGINT, signal_interrupt_handler);
}

std::string ViveLibre::get_config() {
  char * config = vl_get_config(driver->hmd_imu_device);
  return std::string(config);
}

std::map<uint32_t, std::vector<uint32_t>> ViveLibre::poll_angles(std::string channel, uint32_t samples) {
  return driver->poll_angles(channel.at(0), samples);
}

std::pair<std::vector<float>, std::vector<float>> ViveLibre::poll_pnp(std::string channel, uint32_t samples) {
  return driver->poll_pnp(channel.at(0), samples);
}