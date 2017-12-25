/*
This is the PR2-specific version of the robot plugin.
*/
#pragma once

// Headers.
#include <Eigen/Dense>
#include <pluginlib/class_list_macros.h>

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <effort_controllers/joint_effort_controller.h>

// Superclass.
#include "robot_plugin2.h"
#include "gps/proto/gps.pb.h"

// #define REAL_BAXTER

namespace gps_control
{

class GPSPluginExample: public rl_agent::RobotPlugin2, public controller_interface::Controller<
hardware_interface::EffortJointInterface>
{
public:
    // Constructor (this should do nothing).
    GPSPluginExample();
    // Destructor.
    virtual ~GPSPluginExample();
    bool init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& nh);
    void starting(const ros::Time&);
    void stopping(const ros::Time&);
    void update(const ros::Time&, const ros::Duration&);

#ifdef REAL_BAXTER
    // Position command callback.
    virtual void resetSubCb(const rl_msgs::PositionCommand::ConstPtr& msg) override;
#endif

private:
    // PR2-specific chain object necessary to construct the KDL chain.
    // pr2_mechanism_model::Chain passive_arm_chain_, active_arm_chain_;
    // This is a pointer to the robot state(EffortJointInterface), which we get when initialized and have to keep after that.
    hardware_interface::EffortJointInterface* robot_;
    std::vector<hardware_interface::JointHandle> joint_handles_;
    // Create an effort-based joint effort controller for every joint
    // std::vector<boost::shared_ptr<effort_controllers::JointEffortController> > effort_controllers_;
    // Counter for keeping track of controller steps.
    int controller_counter_;
    // Length of controller steps in ms.
    int controller_step_length_;
};

} // end namespace gps_control
