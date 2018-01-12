/*
 * sensor.h
 *
 *  Created on: Dec 22, 2016
 *      Author: silence
 */

#ifndef INCLUDE_RL_AGENT_SENSOR_H_
#define INCLUDE_RL_AGENT_SENSOR_H_

#include <Eigen/Dense>
#include <boost/variant.hpp>
#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include <hardware_interface/joint_command_interface.h>

#include "gps/proto/gps.pb.h"
#include "foundation/label.h"

namespace rl_agent {

// Forward declare
class Sample;

// This is a parameter entry. Note that the arguments should match the enum.
typedef boost::variant<bool*, int*, double*, std::string,
      std::vector<std::string>, Eigen::MatrixXd, Eigen::VectorXd,
      hardware_interface::JointCommandInterface*> DataHandleVariant;
// This is the options map.
typedef std::map<std::string, DataHandleVariant> DataHandleMap;


class Sensor : public Label {
public:
  Sensor(const std::string& name = "SensorDefault");
  virtual bool init() override;

  virtual ~Sensor();
public:
  /*!
   * @brief Only update the sensor data
   */
  virtual void update() = 0;
  virtual void configure(DataHandleMap&) = 0;
  // Register the sensor data handle
  // The sensor handle is referenced by the native pointer
  // The caller need to be careful about free this allocation.
  virtual void configureSample(Sample& sample) = 0;
  /*!
   * @brief Write data into sample at timestamp t
   * @param sample The Sample object
   * @param t      The timestamp t
   */
  virtual void updateSample(Sample& sample, int t) = 0;

protected:
  // Must to be setting in the constructor of subclass
  MiiVector<gps::SampleType> type_include_;
  MiiString                  sensor_name_;
};

} /* namespace rl_agent */

#endif /* INCLUDE_RL_AGENT_SENSOR_H_ */
