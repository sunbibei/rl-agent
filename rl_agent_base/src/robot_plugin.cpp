/*
* robot_plugin.cpp
*
*  Created on: Dec 23, 2016
*      Author: silence
*/

#include "robot_plugin.h"

#include "controller/lqr.h"
#include "controller/position_controller.h"

namespace rl_agent {

////////////////////////////////////////////////////////////////////////////////
/// The main interface
////////////////////////////////////////////////////////////////////////////////
/**
 * The template update workflow
 * Update for controllers, sensors and sample every ticks
 */
void RobotPlugin::updateRobotPlugin(const ros::Time& time, const ros::Duration& period) {
  if (CTRL_PHASE::NOTHING == phase_) return;

  // Update the sensors and the state sample at each time step.
  // ROS_WARN("Update Sensors");
  updateSensors(time, period);
  // Update the controllers and the action sample at each time step.
  // ROS_WARN("Update Controllers");
  updateControllers(time, period);
}

/**
 * Update the sensors at each time step.
 */
void RobotPlugin::updateSensors(const ros::Time& time, const ros::Duration& period) {
  // Update all of the sensors
  // LOG(WARNING) << "Update Sensor";
  sensors_->update();

  // Update the state sample data
  if (nullptr != trial_controller_.get()) {
  // Must to be update sample for next control step
  //  if (!trial_controller_->isFinished())
      ;//sensors_->updateSample(&samples_, trial_controller_->getStepCounter());
  } else {
    ;//sensors_->updateSample(&samples_, 0);
  }


}

/**
 * Update the controllers at each time step.
 */
void RobotPlugin::updateControllers() {
  // reset_controller_->update(state, current, duration, actions_);
  static int trial_step_counter = 0;
  switch (phase_) {
  case CTRL_PHASE::TRIAL_PHASE: // "Trial phase"
  {
    trial_step_counter = trial_controller_->getStepCounter();
    // LOG(WARNING) << "Call GetData to obtain state";
    samples_->getData(trial_controller_->state_types(), X_,
        trial_step_counter);

    // LOG(WARNING) << "The Current Step: " << controller_counter_;
    trial_controller_->control(X_, U_);
    // LOG(WARNING) << "Get Action SUCCESSFUL";
    // Update the sample at each time step.
    samples_->setData(gps::ACTION, U_, U_.size(),
        SampleDataFormat::SampleDataFormatEigenVector,
        trial_step_counter);

    //LOG(WARNING) << "Trial phase";
    if (trial_controller_->isFinished()) { // An episode has finished
      LOG_WARNING << "An Episode has finished, publish ?= "
          << is_report_waiting_;
      if (is_report_waiting_) {
        // LOG(WARNING) << "Is Send Sample?";
        pubSampleReport(samples_, trial_controller_->getTrialLength());
      }

      is_report_waiting_ = false;
      trial_controller_.reset();
      U_.fill(0.0);
      phase_ = CTRL_PHASE::NOTHING;
    }
    break;
  }
  case CTRL_PHASE::RESET_PHASE: // "Reset phase"
  {
    // LOG(WARNING) << "Reset phase";
    if ((reset_controller_->isFinished()) && (is_report_waiting_)) {
      pubSampleReport(samples_, 0);
      LOG_WARNING << "Position target has arrived";
      is_report_waiting_ = false;

      // std::cin.get();
    }
    // LOG(WARNING) << "Reset phase 1";
    // reset_controller_.reset(nullptr); don't need reset to nullptr
    // reset_controller_->reset(current);
    // phase_ = CTRL_PHASE::NOTHING;

    auto types = reset_controller_->state_types();
    /*for (auto& type : types) {
      std::cout << type << " ";
    }
    std::cout << std::endl;*/

    // LOG(WARNING) << "Reset phase 2";
    samples_->getData(reset_controller_->state_types(), X_, 0);
    /*LOG(WARNING) << "Reset phase 3";
    for (int i = 0; i < X_.size(); ++i) {
      std::cout << X_[i] << " ";
    }
    std::cout << std::endl;*/
    reset_controller_->control(X_, U_);
    // LOG(WARNING) << "Reset phase 4";
    // Update the sample at each time step.
    // samples_->setData(gps::ACTION, U_, U_.size(),
    //     SampleDataFormat::SampleDataFormatEigenVector, 0);
    break;
  }
  default:
    LOG_ERROR << "What fucking code!";
    break;
  // Nothing to do here.
  }
}

////////////////////////////////////////////////////////////////////////////////
/// About initialization functions
////////////////////////////////////////////////////////////////////////////////
/**
 * Initialize All variant in the RobotPlugin, especially plugin_loader_
 * Initialization through by the ros parameters. Example as follow:
 * lib_path: /path/to/librl_agent_lib.so
 */
void RobotPlugin::initialize(hardware_interface::JointCommandInterface* robot,
    ros::NodeHandle& nh) {
  LOG(INFO) << "Initializing RobotPlugin";
  robot_ = robot;
  nh_ = ros::NodeHandle(nh, "RobotPlugin");

  ros::NodeHandle nh_topic = ros::NodeHandle(nh_, "topic_name");
  initializeRos(nh_topic);
  ros::NodeHandle nh_sensor = ros::NodeHandle(nh_, "sensors");
  initializeSensors(nh_sensor);

  initializeSamples(nh_);
}
/**
 * Initialize ROS communication infrastructure.
 * Initialization through by the ros parameters. Example as follow:
 * topic_name:
 *  reset:          reset_command
 *  trial:          trial_command
 *  report_request: report_requests
 *  report_pub:     report_pub
 */
void RobotPlugin::initializeRos(ros::NodeHandle& nh) {
  LOG(INFO) << ("Initializing ROS subs/pubs");

  // Create subscribers.
  std::string topic_name = "reset_command";
  if (!nh.getParam("reset", topic_name))
    LOG(WARNING) << "Could not found the reset_topic_name parameter in server, "
        << "using the default topic name: /reset_command";
  reset_sub_ = nh_.subscribe(topic_name, 1, &RobotPlugin::resetSubCb, this);

  topic_name = "trial_command";
  if (!nh.getParam("trial", topic_name))
    LOG(WARNING) << "Could not found the trial_topic_name parameter in server, "
        << "using the default topic name: /trial_command";
  trial_sub_ = nh_.subscribe(topic_name, 1, &RobotPlugin::trialSubCb, this);

  topic_name = "report_request";
  if (!nh.getParam("report_request", topic_name))
    LOG(WARNING) << "Could not found the report_request_topic_name parameter in server, "
        << "using the default topic name: /report_request";
  report_sub_ = nh_.subscribe(topic_name, 1, &RobotPlugin::reportSubCb, this);

  // Create publishers.
  topic_name = "report_pub";
  if (!nh.getParam("report_pub", topic_name))
    LOG(WARNING) << "Could not found the report_pub_topic_name parameter in server, "
        << "using the default topic name: /report_pub";
  report_pub_.reset(
      new realtime_tools::RealtimePublisher<SampleResult>(nh_, topic_name, 1));
}

/**
 * Initialize all sensors.
 * Factory Method, Create all of the sensor through by the ros parameter
 * Example as follow:
 * sensors:
 *  types: # list the class name of all sensor
 *    - JointEncoder
 *    - ChainJointEncoder
 *    - IMU
 *    - xxx
 *  configures:
 *    JointEncoder: # list the names of all encoder
 *      - hip
 *      - knee
 *      - x_axis
 *      - z_axis
 *    ChainJointEncoder:
 *      chain: {root_link: torso_lift_link, tip_link: l_gripper_tool_frame}
 *      joints:
 *        - l_shoulder_pan_joint
 *        - l_shoulder_lift_joint
 *        - l_upper_arm_roll_joint
 *        - l_elbow_flex_joint
 *        - l_forearm_roll_joint
 *        - l_wrist_flex_joint
 *        - l_wrist_roll_joint
 *    IMU: # list the names of all imu
 *      - imu_1
 *      - imu_2
 *    xxx: # list the names of all xxx
 *      - xxx
 */
void RobotPlugin::initializeSensors(ros::NodeHandle& nh) {
//  std::string lib_path;
//  if (!nh.getParam("lib_path", lib_path)) {
//    LOG(FATAL) << "Could not found the lib_path parameter in server, "
//        << "' (namespace: " << nh.getNamespace() << ").";
//    return;
//  }
//  plugin_loader_.reset(new class_loader::ClassLoader(lib_path));
//
//  sensors_.reset(new Sensor("Sensor Composite"));
//
//  const int NS_SIZE = 10; // "rl_agent::"  total size: 10
//  std::vector<std::string> sensor_types; // List of sensor_name
//  std::string param_name = "types";
//  if(!nh.getParam(param_name, sensor_types)) {
//    LOG(ERROR) << "Failed to getParam '" << param_name
//        << "' (namespace: " << nh.getNamespace() << ").";
//    return;
//  }
//
//  std::vector<std::string> ava_list = plugin_loader_->getAvailableClasses<Sensor>();
//  std::vector<std::string> loader_list;
//  for (const auto& t_param : sensor_types) {
//    bool is_find = false;
//    for (const auto& a_loader : ava_list) {
//      LOG(INFO) << "Available sensor classes: " << a_loader;
//      // ignore the frist 10 characters (rl_agent::)
//      if (0 == t_param.compare(a_loader.substr(NS_SIZE))) {
//        is_find = true;
//        // Collecting the available classes list
//        LOG(INFO) << "The type (" << t_param << ") of sensor your given has found!";
//        loader_list.push_back(a_loader);
//        break;
//      }
//    }
//    if (!is_find) {
//      LOG(ERROR) << "The type (" << t_param << ") of sensor your given, "
//          << "Can't load by class_loader, Did you forget define this sensor?";
//    }
//  }
//
//  ros::NodeHandle nh_config = ros::NodeHandle(nh, "configures/");
//  for (const auto& type : loader_list) {
//    param_name = type.substr(NS_SIZE) + "/names";
//    std::vector<std::string> sensor_names;
//    if(!nh_config.getParam(param_name, sensor_names)) {
//      LOG(WARNING) << "Failed to getParam '" << param_name
//          << "' (namespace: " << nh.getNamespace() << ").";
//      continue;
//    }
//    /*
//    boost::shared_ptr<Sensor> sensor = plugin_loader_->createInstance<Sensor>(type);
//    sensor->setName(type.substr(NS_SIZE));
//    sensors_->add(sensor);
//    */
//    Sensor* types = new Sensor(type.substr(NS_SIZE));
//    for (const auto& name : sensor_names) {
//      boost::shared_ptr<Sensor> sensor = plugin_loader_->createInstance<Sensor>(type);
//      sensor->setName(name);
//      types->add(sensor);
//    }
//    sensors_->add(types);
//  }
//
//  DataHandleMap opts;
//  opts["robot_handle"] = robot_;
//  sensors_->configureSensor(nh_config, opts);
//  LOG(INFO) << "Sensors have configured!";
}

void RobotPlugin::initializeSamples(ros::NodeHandle& nh) {
  samples_.reset(new Sample(1)); // The default T = 1
  sensors_->configureSample(*samples_);
  LOG(INFO) << "Sample has configured!";
}

////////////////////////////////////////////////////////////////////////////////
/// About ROS Communication functions
////////////////////////////////////////////////////////////////////////////////
// Report publishers
// Publish a sample with data from up to T timesteps
void RobotPlugin::pubSampleReport(boost::scoped_ptr<Sample>& sample, int T) {
  while(!report_pub_->trylock());

  if (T > 0) {
    std::vector<gps::SampleType> dtypes;
    sample->getAvailableDataTypes(dtypes);

    report_pub_->msg_.sensor_data.resize(dtypes.size());
    for(size_t d = 0; d < dtypes.size(); d++) { //Fill in each sample type
      report_pub_->msg_.sensor_data[d].data_type = dtypes[d];
      Eigen::VectorXd tmp_data;
      sample->getDataList(dtypes[d], tmp_data, T);
      report_pub_->msg_.sensor_data[d].data.resize(tmp_data.size());
      std::vector<int> shape;
      sample->getShape(dtypes[d], shape);
      shape.insert(shape.begin(), T);
      report_pub_->msg_.sensor_data[d].shape.resize(shape.size());
      int total_expected_shape = 1;
      for (size_t i = 0; i < shape.size(); ++i) {
        report_pub_->msg_.sensor_data[d].shape[i] = shape[i];
        total_expected_shape *= shape[i];
      }
      if (total_expected_shape != tmp_data.size()) {
        LOG(ERROR) << "Data stored in sample has different length than expected ("
            << tmp_data.size() << " vs "
            << total_expected_shape << ")";
      }
      for (int i = 0; i < tmp_data.size(); ++i) {
        report_pub_->msg_.sensor_data[d].data[i] = tmp_data[i];
      }
    }
  }

  report_pub_->unlockAndPublish();
}

// Subscriber callbacks.
// Position command callback.
void RobotPlugin::resetSubCb(const PositionCommand::ConstPtr& msg) {
  LOG(INFO) << "received position command";
  phase_ = CTRL_PHASE::NOTHING;

  CtrlConfigMap params;
  params["mode"] = msg->mode;
  Eigen::VectorXd data;
  data.resize(msg->data.size());
  for(int i=0; i< data.size(); ++i) {
    data[i] = msg->data[i];
  }
  params["data"] = data;

  Eigen::MatrixXd pd_gains;
  // LOG(WARNING) << "data size: " << data.size();
  // LOG(WARNING) << "pd_gains size: " << msg->pd_gains.size();
  pd_gains.resize(msg->pd_gains.size() / 4, 4);
  for(int i = 0; i < pd_gains.rows(); ++i) {
    for(int j = 0; j < 4; ++j) {
      pd_gains(i, j) = msg->pd_gains[i * 4 + j];
    }
  }
  params["pd_gains"] = pd_gains;
  std::vector<gps::SampleType> state_inc;
  state_inc.push_back(gps::JOINT_ANGLES);
  params["state_include"] = state_inc;

  if (nullptr == reset_controller_.get()) {
    reset_controller_.reset(new PositionController("Reset Controller"));
  }

  reset_controller_->reset();
  reset_controller_->configure(params);
  reset_controller_->configureSample(*samples_);
  LOG(INFO) << "configure Sample and Reset Controller SUCCESSFUL!";

  is_report_waiting_ = true;
  phase_ = CTRL_PHASE::RESET_PHASE;
}

// Trial command callback.
void RobotPlugin::trialSubCb(const TrialCommand::ConstPtr& msg) {
  LOG(INFO) << "received trial command";
  phase_ = CTRL_PHASE::NOTHING;

  CtrlConfigMap controller_params;
  std::vector<gps::SampleType> state_datatypes;
  state_datatypes.reserve(msg->state_datatypes.size());
  for(const auto& type : msg->state_datatypes) {
    state_datatypes.push_back((gps::SampleType)type);
    // LOG(WARNING) << "state_define: " << type;
  }
  controller_params["state_include"] = state_datatypes;

  std::vector<gps::SampleType> obs_datatypes;
  obs_datatypes.reserve(msg->obs_datatypes.size());
  for(const auto& type : obs_datatypes) {
    obs_datatypes.push_back(type);
  }
  controller_params["obs_include"] = obs_datatypes;
  // msg::frequency in Hz
  controller_params["ctrl_frequency"] = (1.0 / msg->frequency);
  // LOG(ERROR) << "MSG->FREQUENCY: " << msg->frequency;
  //controller_params["X_include"] = state_inc;

  if(msg->controller.controller_to_execute == gps::LIN_GAUSS_CONTROLLER) {
    configureLqr(msg, controller_params);
  }
#ifdef USE_CAFFE
  else if (msg->controller.controller_to_execute == gps::CAFFE_CONTROLLER) {
    configureCaffePolicy(msg, controller_params);
  }
#endif
#ifdef USE_TF
  else if (msg->controller.controller_to_execute == gps::TF_CONTROLLER) {
    configureTfPolicy(msg, controller_params);
  }
#endif
  else {
    LOG(ERROR) << "Unknown trial controller arm type and/or USE_CAFFE=0/or USE_TF=0";
  }

  // TODO What is funking code!
  // Configure sensor for trial
  DataHandleMap sensor_params;
  // Feed EE points/sites to sensors
  Eigen::MatrixXd ee_points;
  if(0 != msg->ee_points.size() % 3) {
    LOG(ERROR) << "Got " << msg->ee_points.size()
        << " %d ee_points (must be multiple of 3)";
  }
  int n_points = msg->ee_points.size() / 3;
  ee_points.resize(n_points, 3);
  for(int i = 0; i < n_points; ++i){
    for(int j = 0; j < 3; ++j){
      ee_points(i, j) = msg->ee_points[j + 3 * i];
    }
  }
  sensor_params["ee_sites"] = ee_points;

  // update end effector points target
  Eigen::MatrixXd ee_points_tgt;
  if(msg->ee_points_tgt.size() != ee_points.size()) {
    LOG(ERROR) << "Got " << msg->ee_points_tgt.size()
        << "ee_points_tgt (must match ee_points size: "
        << msg->ee_points.size() << ")";
  }
  ee_points_tgt.resize(n_points, 3);
  for(int i=0; i<n_points; i++) {
    for(int j=0; j<3; j++) {
      ee_points_tgt(i, j) = msg->ee_points_tgt[j+3*i];
    }
  }
  sensor_params["ee_points_tgt"] = ee_points_tgt;
  configureSensors(sensor_params);

  //Read out trial information
  if (msg->T > MAX_TRIAL_LENGTH)
    LOG(FATAL) << "Trial length specified is longer than maximum trial length ("
        << msg->T << " vs " << MAX_TRIAL_LENGTH <<")";

  configureSamples(msg->T);
  // Update sensor frequency
  // sensors_->setUpdateFrequency(msg->frequency);
  phase_ = CTRL_PHASE::TRIAL_PHASE;
  is_report_waiting_ = true;
}

// Helper method to configure all sensors
void RobotPlugin::configureSensors(DataHandleMap& opts) {
  LOG(INFO) << "configure sensors";
  // TODO
  // sensors_->configureSensor(nh_, opts);
}

void RobotPlugin::configureSamples(int T) {
  if (nullptr == samples_.get()) {
    LOG(INFO) << "Get the instance of sample, using T = "
        << T;
    samples_.reset(new Sample(T));
  } else {
    samples_->resize(T);
    LOG(WARNING) << "Resize Sample OK!";
  }

  // LOG(WARNING) << "Ready to configure Sample";
  sensors_->configureSample(*samples_);
  // LOG(WARNING) << "Configure Sample SUCCESSFUL";
  if (nullptr != trial_controller_.get()) {
    LOG(INFO) << "Trial Controller is configuring samples";
    trial_controller_->configureSample(*samples_);
    LOG(WARNING) << "Trial Controller has configured samples";
  } else if (nullptr != reset_controller_.get()) {
    LOG(INFO) << "Reset Controller is configuring samples";
    reset_controller_->configureSample(*samples_);
  } else {
    LOG(WARNING) << "Configure the sample before instancing controller"
        << "(reset or trial controller), using the default value ("
        << "the number of joints).";
    // Set sample data format on the actions, which are not handled by any sensor.
    OptionsMap sample_metadata;
    samples_->setMetaData(gps::ACTION, U_.size(),
        SampleDataFormat::SampleDataFormatEigenVector, sample_metadata);
  }
}

void RobotPlugin::configureLqr(
    const TrialCommand::ConstPtr& msg, CtrlConfigMap& params) {
  LinGaussParams lingauss = msg->controller.lingauss;
  int dX = (int) lingauss.dX;
  int dU = (int) lingauss.dU;
  // LOG(WARNING) << "dX: " << dX << ", dU: " << dU;
  //Prepare options map
  params["T"] = (int)msg->T;
  params["dX"] = dX;
  params["dU"] = dU;
  for(int t = 0; t < (int)msg->T; ++t) {
    Eigen::MatrixXd K;
    K.resize(dU, dX);
    for(int u = 0; u < dU; ++u) {
      for(int x = 0; x < dX; ++x){
        K(u, x) = lingauss.K_t[x + u * dX + t * dU * dX];
      }
    }
    Eigen::VectorXd k;
    k.resize(dU);
    for(int u=0; u<dU; u++){
      k(u) = lingauss.k_t[u + t*dU];
    }
    params["K_" + std::to_string(t)] = K;
    params["k_" + std::to_string(t)] = k;
  }
  trial_controller_.reset(new LQR("LQR"));
  trial_controller_->configure(params);
}

inline void RobotPlugin::configureCaffePolicy(
    const TrialCommand::ConstPtr& msg, CtrlConfigMap& params) {
  bool is_def_caffe = false;
#ifdef USE_CAFFE
  is_define = true;
  CaffeParams params = msg->controller.caffe;
  CaffeNNController* ctrller = new CaffeNNController();

  // TODO(chelsea/zoe): put this somewhere else.
  int dim_bias = params.dim_bias;
  Eigen::MatrixXd scale;
  scale.resize(dim_bias, dim_bias);
  Eigen::VectorXd bias;
  bias.resize(dim_bias);

  int dU = (int) params.dU;
  int idx = 0;
  // Unpack the scale matrix
  for (int j = 0; j < dim_bias; ++j) {
    for (int i = 0; i < dim_bias; ++i) {
      scale(i,j) = params.scale[idx];
      idx++;
    }
  }

  idx = 0;
  // Unpack the bias vector
  for (int i = 0; i < dim_bias; ++i) {
    bias(i) = params.bias[idx];
    idx++;
  }

  for(int t=0; t<(int)msg->T; t++){
    Eigen::VectorXd noise;
    noise.resize(dU);
    for(int u=0; u<dU; u++) {
      noise(u) = params.noise[u+t*dU];
    }
    controller_params["noise_"+to_string(t)] = noise;
  }

  params["net_param"] = params.net_param;
  params["scale"] = scale;
  params["bias"] = bias;
  params["T"] = (int)msg->T;
  ctrller->configure(params);
  trial_controller_.reset(ctrller);
#endif
  if (!is_def_caffe) {
    LOG(ERROR) << "NO DEFINE USE_CAFFE";
  }
}

inline void RobotPlugin::configureTfPolicy(
    const TrialCommand::ConstPtr& msg, CtrlConfigMap& params) {
  bool is_def_tf = false;
#ifdef USE_TF
  is_def_tf = true;
  TfController* ctrller = new TfController();
  params["T"] = (int)msg->T;
  TfParams tfparams = msg->controller.tf;
  int dU = (int) tfparams.dU;
  params["dU"] = dU;
  ctrller->configure(params);
  trial_controller_.reset(ctrller);
#endif
  if (!is_def_tf) {
    LOG(ERROR) << "NO DEFINE USE_TF";
  }
}

// Data request callback.
void RobotPlugin::reportSubCb(const DataRequest::ConstPtr& msg) {
  LOG(INFO) << ("received data request");
  is_report_waiting_ = true;
}

} /* namespace rl_agent */
