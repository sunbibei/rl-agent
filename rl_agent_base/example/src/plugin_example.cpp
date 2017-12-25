#include "plugin_example.h"

#include <glog/logging.h>

namespace gps_control {

// Plugin constructor.
GPSPluginExample::GPSPluginExample() {
    // Some basic variable initialization.
    controller_counter_ = 0;
    controller_step_length_ = 50;
    google::InitGoogleLogging("RobotPlugin");
    //google::LogToStderr();
    //google::SetLogDestination(google::GLOG_INFO, "/home/silence/log/INFO_");
    //google::SetLogDestination(google::GLOG_WARNING, "/home/silence/log/WARNING_");
    //google::SetLogDestination(google::GLOG_ERROR, "/home/silence/log/ERROR_");
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold=google::INFO;
    FLAGS_logtostderr = true;
    //google::FlushLogFiles(google::GLOG_INFO);
}

// Destructor.
GPSPluginExample::~GPSPluginExample()
{
    google::ShutdownGoogleLogging();
    // Nothing to do here, since all instance variables are destructed automatically.
}

// Initialize the object and store the robot state.
bool GPSPluginExample::init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& n)
{
    ROS_WARN("Silence: controller init() is called!");
    initialize();
    LOG(INFO) << ("RobotPlugin initialize successful!");

    robot_ = robot; // store the pointer
    // Put together joint states for the active arm.
    LOG(INFO) << "Namespace: " << n.getNamespace();
    int joint_index = 1;
    while (true) {
        // Check if the parameter for this active joint exists.
        std::string joint_name;
        std::string param_name = std::string("joint_name_" + std::to_string(joint_index));
        if(!n.getParam(param_name, joint_name))
            break;

        // Push back the joint state and name.
        LOG(INFO) << "Push Back The joint name: " << joint_name;
        joint_handles_.push_back(robot_->getHandle(joint_name));
        // Increment joint index.
        joint_index++;
    }
    U_.resize(joint_handles_.size());
    U_.fill(0.0);
    // ROS_WARN("Silence: controller requestStart() is called!");
    return true;
}

// This is called by the controller manager before starting the controller.
void GPSPluginExample::starting(const ros::Time& time)
{
    ROS_WARN("Silence: controller starting() is called!");
    controller_counter_ = 0;
}

// This is called by the controller manager before stopping the controller.
void GPSPluginExample::stopping(const ros::Time& time)
{
    // Nothing to do here.
}

// This is the main update function called by the realtime thread when the controller is running.
void GPSPluginExample::update(const ros::Time& time, const ros::Duration& period)
{
    // Call RobotPlugin::update
    updateRobotPlugin();

    // Store the torques.
    //std::cout << "torques: ";
    for (unsigned i = 0; i < joint_handles_.size(); i++) {
        joint_handles_[i].setCommand(U_[i]);
        //std::cout << U_[i] << " ";
    }
    //std::cout << std::endl;
}

#ifdef REAL_BAXTER
// Position command callback.
void GPSPluginExample::resetSubCb(const rl_msgs::PositionCommand::ConstPtr& msg) {
  LOG(INFO) << "Baxter: received position command";
  phase_ = rl_agent::CTRL_PHASE::NOTHING;

  rl_agent::CtrlConfigMap params;
  Eigen::VectorXd data;
  data.resize(msg->data.size());
  for(int i=0; i< data.size(); ++i) {
    data[i] = msg->data[i];
  }
  params["data"] = data;

  std::vector<std::string> jnt_names;
  for (int i = 0; i < joint_handles_.size(); ++i) {
    jnt_names.push_back(joint_handles_[i].getName());
  }
  params["jnt_names"] = jnt_names;

  if (nullptr == reset_controller_.get()) {
    reset_controller_.reset(new rl_agent::BaxterResetController("Reset Controller"));
  }

  reset_controller_->configure(params);
  reset_controller_->configureSample(samples_);
  LOG(INFO) << "configure Sample and Reset Controller SUCCESSFUL!";

  is_report_waiting_ = true;
  phase_ = rl_agent::CTRL_PHASE::RESET_PHASE;
}
#endif

} // end namespace gps_control

// Register controller to pluginlib
// PLUGINLIB_DECLARE_CLASS(gps_agent_pkg, GPSPluginExample,
// 						gps_control::GPSPluginExample,
// 						controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS( gps_control::GPSPluginExample, controller_interface::ControllerBase )
