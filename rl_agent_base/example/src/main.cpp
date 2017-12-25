/*
 * main.cpp
 *
 *  Created on: Dec 21, 2017
 *      Author: bibei
 */

#include "plugin_example.h"
#include <ros/ros.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "test_rl_agent_base");

  gps_control::GPSPluginExample* plugin = new gps_control::GPSPluginExample;

  ros::Duration elapsed_time;
  struct timespec last_time, current_time;
  static const double BILLION = 1000000000.0;
  clock_gettime(CLOCK_MONOTONIC, &last_time);

  while (ros::ok()) {
    // Control
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    elapsed_time = ros::Duration(current_time.tv_sec - last_time.tv_sec
        + (current_time.tv_nsec - last_time.tv_nsec) / BILLION);

    plugin->update(ros::Time::now(), elapsed_time);
    last_time = current_time;
  }

  return 0;
}


