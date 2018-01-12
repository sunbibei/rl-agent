/*
 * robot_plugin.h
 *
 *  Created on: Dec 23, 2016
 *      Author: silence
 */

#ifndef INCLUDE_RL_AGENT_ROBOT_PLUGIN2_H_
#define INCLUDE_RL_AGENT_ROBOT_PLUGIN2_H_

#include <atomic>
#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>

#include "rl_msgs/SampleResult.h"
#include "rl_msgs/PositionCommand.h"
#include "rl_msgs/TrialCommand.h"
#include "rl_msgs/DataRequest.h"

#include "controller/trial_controller.h"
#include "controller/controller.h"
#include "sensor/sensor.h"
#include "sample/sample.h"

#include <foundation/utf.h>

namespace rl_agent {

using namespace rl_msgs;

const int MAX_TRIAL_LENGTH = 2000;

enum class RL_STATE {
  UNKONWN_RL_STATE = -1,
  RL_STATE_NOTHING = 0,
  RL_STATE_RESET,
  RL_STATE_TRIAL,
  N_RL_STATE,
};

class RobotPlugin2 {
public:
  RobotPlugin2(const MiiString& _l = "RobotPlugin2");
  virtual ~RobotPlugin2();

public:
  virtual void initialize();

  virtual void updateRobotPlugin();

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
  virtual void initializeRos(const MiiString&);
  // Initialize all sensors.
  virtual void initializeSensors(const MiiString&);

  virtual void initializeSamples();
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
  MiiString tag_;

  RL_STATE current_state_;

  /// Subscriber
  // for reset/trial/report command
  ros::Subscriber reset_sub_;
  ros::Subscriber trial_sub_;
  ros::Subscriber report_sub_;

  /// Publisher
  // for result sample message
  boost::scoped_ptr<realtime_tools::RealtimePublisher<SampleResult>> report_pub_;

  // Controllers
  boost::shared_ptr<Controller>       reset_controller_;
  boost::shared_ptr<TrialController>  trial_controller_;
  // Sensors
  Sensor* sensors_;
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
