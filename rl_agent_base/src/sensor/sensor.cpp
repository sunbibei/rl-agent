/*
 * sensor.h
 *
 *  Created on: Dec 22, 2016
 *      Author: silence
 */

#include "sensor/sensor.h"
#include <foundation/cfg_reader.h>

namespace rl_agent {

Sensor::Sensor(const std::string& name)
  : Label(name) { };

Sensor::~Sensor() { };

bool Sensor::init() {
  auto cfg = MiiCfgReader::instance();
  cfg->get_value(getLabel(), "name", sensor_name_);
  return true;
}

} /* namespace rl_agent */
