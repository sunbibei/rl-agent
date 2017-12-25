/*
 * robot_plugin.h
 *
 *  Created on: Dec 23, 2016
 *      Author: silence
 */

#ifndef INCLUDE_RL_AGENT_ROBOT_PLUGIN_H_
#define INCLUDE_RL_AGENT_ROBOT_PLUGIN_H_

#include <atomic>
#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>
#include <class_loader/class_loader.h>
#include <realtime_tools/realtime_publisher.h>

#include "rl_msgs/SampleResult.h"
#include "rl_msgs/PositionCommand.h"
#include "rl_msgs/TrialCommand.h"
#include "rl_msgs/DataRequest.h"

#include "controller/trial_controller.h"
#include "controller/controller.h"
#include "sensor/sensor.h"
#include "sample/sample.h"

#include <glog/logging.h>

namespace rl_agent {

using namespace rl_msgs;

const int MAX_TRIAL_LENGTH = 2000;

enum class CTRL_PHASE {
  NOTHING = 0,
  RESET_PHASE,
  TRIAL_PHASE
};

class RobotPlugin {
public:
  RobotPlugin()
    : robot_(nullptr), phase_(CTRL_PHASE::NOTHING), plugin_loader_(nullptr),
      is_report_waiting_(false)/*, is_sensors_initialized_(false)*/ {
    ;// Nothing to do here
  }
  virtual ~RobotPlugin() {
    ;
  }

  virtual void initialize(hardware_interface::JointCommandInterface* robot,
      ros::NodeHandle& nh);

  virtual void updateRobotPlugin(const ros::Time& current, const ros::Duration& duration);

protected:
////////////////////////////////////////////////////////////////////////////////
/// Update functions.
////////////////////////////////////////////////////////////////////////////////
  // Update the controllers at each time step.
  virtual void updateControllers();
  // Update the sensors at each time step.
  virtual void updateSensors();

////////////////////////////////////////////////////////////////////////////////
/// About initialization functions
////////////////////////////////////////////////////////////////////////////////
  // Initialize ROS communication infrastructure.
  virtual void initializeRos(ros::NodeHandle& nh);
  // Initialize all sensors.
  virtual void initializeSensors(ros::NodeHandle& nh);

  virtual void initializeSamples(ros::NodeHandle& nh);
////////////////////////////////////////////////////////////////////////////////
/// About ROS Communication functions
////////////////////////////////////////////////////////////////////////////////
  // Report publishers
  // Publish a sample with data from up to T timesteps
  virtual void pubSampleReport(boost::scoped_ptr<Sample>& sample, int T = 1);

  // Subscriber callbacks.
  // Position command callback.
  virtual void resetSubCb(const PositionCommand::ConstPtr& msg);
  // Trial command callback.
  virtual void trialSubCb(const TrialCommand::ConstPtr& msg);
  // Data request callback.
  virtual void reportSubCb(const DataRequest::ConstPtr& msg);

/// Helper Methods
protected:
  void configureLqr(const TrialCommand::ConstPtr&, CtrlConfigMap&);
  void configureCaffePolicy(const TrialCommand::ConstPtr&, CtrlConfigMap&);
  void configureTfPolicy(const TrialCommand::ConstPtr&, CtrlConfigMap&);

    // samples and sensors
  void configureSamples(int T = 1);
  void configureSensors(DataHandleMap&);

protected:
  ros::NodeHandle nh_;
  hardware_interface::JointCommandInterface* robot_;
  CTRL_PHASE phase_;
  /// Subscriber
  // for reset/trial/report command
  ros::Subscriber reset_sub_;
  ros::Subscriber trial_sub_;
  ros::Subscriber report_sub_;

  /// Publisher
  // for result sample message
  boost::scoped_ptr<realtime_tools::RealtimePublisher<SampleResult>> report_pub_;

  /// Instance
  boost::scoped_ptr<class_loader::ClassLoader> plugin_loader_;
  // Controllers
  boost::scoped_ptr<Controller>       reset_controller_;
  boost::scoped_ptr<TrialController>  trial_controller_;

  // Sensors
  boost::scoped_ptr<Sensor> sensors_;
  // Sample
  boost::scoped_ptr<Sample> samples_;

  // Action Vector
  Eigen::VectorXd U_;
  Eigen::VectorXd X_;

  /// Flag
  // Is a trial data request pending?
  std::atomic_bool is_report_waiting_;
};

} /* namespace rl_agent */

#endif /* INCLUDE_RL_AGENT_ROBOT_PLUGIN_H_ */
