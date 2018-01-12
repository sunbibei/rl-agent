/*
* robot_plugin.cpp
*
*  Created on: Dec 23, 2016
*      Author: silence
*/

#include "robot_plugin2.h"

#include "controller/lqr.h"
#include "controller/position_controller.h"

#include <foundation/cfg_reader.h>

namespace rl_agent {

RobotPlugin2::RobotPlugin2(const MiiString& _l)
  : tag_(_l), current_state_(RL_STATE::RL_STATE_NOTHING),
    reset_controller_(nullptr), trial_controller_(nullptr),
    sensors_(nullptr), samples_(nullptr),
    is_report_waiting_(false)/*, is_sensors_initialized_(false)*/ {
  ; // Nothing to do here
  nh_ = ros::NodeHandle("RLAgentBase");
}

////////////////////////////////////////////////////////////////////////////////
/// About initialization functions
////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Initialize All variant in the RobotPlugin2
 */
void RobotPlugin2::initialize() {
  LOG_INFO << "Initializing RobotPlugin2";

  MiiString tag = Label::make_label(tag_, "topic");
  initializeRos(tag);

  tag = Label::make_label(tag_, "sensor");
  initializeSensors(tag);

  initializeSamples();
}

void RobotPlugin2::initializeRos(const MiiString& topic_tag) {
  LOG_INFO << ("Initializing ROS subs/pubs");

  auto cfg = MiiCfgReader::instance();
  // Create subscribers.
  MiiString topic_name = "reset_command";
  if (!cfg->get_value(topic_tag, "reset", topic_name))
    LOG_WARNING << "Could not found the reset_topic_name parameter in tag("
        << topic_tag << "), using the default topic name: /reset_command";
  reset_sub_ = nh_.subscribe(topic_name, 1, &RobotPlugin2::resetSubCb, this);

  topic_name = "trial_command";
  if (!cfg->get_value(topic_tag, "trial", topic_name))
    LOG_WARNING << "Could not found the trial_topic_name parameter in tag("
        << topic_tag << "), using the default topic name: /trial_command";
  trial_sub_ = nh_.subscribe(topic_name, 1, &RobotPlugin2::trialSubCb, this);

  topic_name = "report_request";
  if (!cfg->get_value(topic_tag, "report_request", topic_name))
    LOG_WARNING << "Could not found the report_request_topic_name parameter in tag("
        << topic_tag << "), using the default topic name: /report_request";
  report_sub_ = nh_.subscribe(topic_name, 1, &RobotPlugin2::reportSubCb, this);

  // Create publishers.
  topic_name = "report_pub";
  if (!cfg->get_value(topic_tag, "report_pub", topic_name))
    LOG_WARNING << "Could not found the report_pub_topic_name parameter in tag("
        << topic_tag << "), using the default topic name: /report_pub";
  report_pub_.reset(
      new realtime_tools::RealtimePublisher<SampleResult>(nh_, topic_name, 1));
}

void RobotPlugin2::initializeSensors(const MiiString& sensor_tag) {
  auto cfg = MiiCfgReader::instance();

  MiiString sensor;
  cfg->get_value_fatal(sensor_tag, "label", sensor);
  sensors_ = Label::getHardwareByName<Sensor>(sensor);

  LOG_INFO << "Sensors have configured!";
}

void RobotPlugin2::initializeSamples() {
  samples_.reset(new Sample(1)); // The default T = 1
  sensors_->configureSample(*samples_);
  LOG_INFO << "Sample has configured!";
}

RobotPlugin2::~RobotPlugin2() {
  ;
}

////////////////////////////////////////////////////////////////////////////////
/// The main interface
////////////////////////////////////////////////////////////////////////////////
/**
 * The template update workflow
 * Update for controllers, sensors and sample every ticks
 */
void RobotPlugin2::updateRobotPlugin() {
  if (RL_STATE::RL_STATE_NOTHING == current_state_) return;

  // Update the sensors and the state sample at each time step.
  // ROS_WARN("Update Sensors");
  updateSensors();
  // Update the controllers and the action sample at each time step.
  // ROS_WARN("Update Controllers");
  updateControllers();
}

/**
 * Update the sensors at each time step.
 */
void RobotPlugin2::updateSensors() {
  // Update all of the sensors
  // LOG_WARNING << "Update Sensor";
  sensors_->update();

  // Update the state sample data
  if (nullptr != trial_controller_) {
  // Must to be update sample for next control step
  //  if (!trial_controller_->isFinished())
      sensors_->updateSample(*samples_, trial_controller_->getStepCounter());
  } else {
    sensors_->updateSample(*samples_, 0);
  }
}

/**
 * Update the controllers at each time step.
 */
void RobotPlugin2::updateControllers() {
  // reset_controller_->update(state, current, duration, actions_);
  static int trial_step_counter = 0;
  switch (current_state_) {
  case RL_STATE::RL_STATE_TRIAL: // "Trial phase"
  {
    trial_step_counter = trial_controller_->getStepCounter();
    // LOG_WARNING << "Call GetData to obtain state";
    samples_->getData(trial_controller_->state_types(), X_,
        trial_step_counter);

    // LOG_WARNING << "The Current Step: " << controller_counter_;
    trial_controller_->control(X_, U_);
    // LOG_WARNING << "Get Action SUCCESSFUL";
    // Update the sample at each time step.
    samples_->setData(gps::ACTION, U_, U_.size(),
        SampleDataFormat::SampleDataFormatEigenVector,
        trial_step_counter);

    // LOG_WARNING << "Trial phase";
    if (trial_controller_->isFinished()) { // An episode has finished
      LOG_WARNING << "An Episode has finished, publish ?= "
          << is_report_waiting_;
      if (is_report_waiting_) {
        // LOG_WARNING << "Is Send Sample?";
        pubSampleReport(samples_, trial_controller_->getTrialLength());
      }

      is_report_waiting_ = false;
      trial_controller_.reset();

      U_.fill(0.0);
      current_state_ = RL_STATE::RL_STATE_NOTHING;
    }
    break;
  }
  case RL_STATE::RL_STATE_RESET: // "Reset phase"
  {
    // LOG_WARNING << "Reset phase";
    if ((reset_controller_->isFinished()) && (is_report_waiting_)) {
      pubSampleReport(samples_, 0);
      LOG_WARNING << "Position target has arrived";
      is_report_waiting_ = false;

      // std::cin.get();
    }
    // LOG_WARNING << "Reset phase 1";
    // reset_controller_.reset(nullptr); don't need reset to nullptr
    // reset_controller_->reset(current);
    // phase_ = CTRL_PHASE::NOTHING;

    auto types = reset_controller_->state_types();
    /*for (auto& type : types) {
      std::cout << type << " ";
    }
    std::cout << std::endl;*/

    // LOG_WARNING << "Reset phase 2";
    samples_->getData(reset_controller_->state_types(), X_, 0);
    /*LOG_WARNING << "Reset phase 3";
    for (int i = 0; i < X_.size(); ++i) {
      std::cout << X_[i] << " ";
    }
    std::cout << std::endl;*/
    reset_controller_->control(X_, U_);
    // LOG_WARNING << "Reset phase 4";
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
/// About ROS Communication functions
////////////////////////////////////////////////////////////////////////////////
// Report publishers
// Publish a sample with data from up to T timesteps
void RobotPlugin2::pubSampleReport(boost::scoped_ptr<Sample>& sample, int T) {
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
        LOG_ERROR << "Data stored in sample has different length than expected ("
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
void RobotPlugin2::resetSubCb(const PositionCommand::ConstPtr& msg) {
  LOG_INFO << "received position command";
  current_state_ = RL_STATE::RL_STATE_NOTHING;

  CtrlConfigMap params;
  params["mode"] = msg->mode;
  Eigen::VectorXd data;
  data.resize(msg->data.size());
  for(int i=0; i< data.size(); ++i) {
    data[i] = msg->data[i];
  }
  params["data"] = data;

  Eigen::MatrixXd pd_gains;
  // LOG_WARNING << "data size: " << data.size();
  // LOG_WARNING << "pd_gains size: " << msg->pd_gains.size();
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
  LOG_INFO << "configure Sample and Reset Controller SUCCESSFUL!";

  is_report_waiting_ = true;
  current_state_ = RL_STATE::RL_STATE_RESET;
}

// Trial command callback.
void RobotPlugin2::trialSubCb(const TrialCommand::ConstPtr& msg) {
  LOG_INFO << "received trial command";
  current_state_ = RL_STATE::RL_STATE_NOTHING;

  CtrlConfigMap controller_params;
  std::vector<gps::SampleType> state_datatypes;
  state_datatypes.reserve(msg->state_datatypes.size());
  for(const auto& type : msg->state_datatypes) {
    state_datatypes.push_back((gps::SampleType)type);
    // LOG_WARNING << "state_define: " << type;
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
  // LOG_ERROR << "MSG->FREQUENCY: " << msg->frequency;
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
    LOG_ERROR << "Unknown trial controller arm type and/or USE_CAFFE=0/or USE_TF=0";
  }

  // TODO What is funking code!
  // Configure sensor for trial
  DataHandleMap sensor_params;
  // Feed EE points/sites to sensors
  Eigen::MatrixXd ee_points;
  if(0 != msg->ee_points.size() % 3) {
    LOG_ERROR << "Got " << msg->ee_points.size()
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
    LOG_ERROR << "Got " << msg->ee_points_tgt.size()
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
    LOG_FATAL << "Trial length specified is longer than maximum trial length ("
        << msg->T << " vs " << MAX_TRIAL_LENGTH <<")";

  configureSamples(msg->T);
  // Update sensor frequency
  // sensors_->setUpdateFrequency(msg->frequency);
  current_state_ = RL_STATE::RL_STATE_TRIAL;
  is_report_waiting_ = true;
}

// Helper method to configure all sensors
void RobotPlugin2::configureSensors(DataHandleMap& opts) {
  LOG_WARNING << "configure sensors";
  // TODO
  sensors_->configure(opts);
}

void RobotPlugin2::configureSamples(int T) {
  if (nullptr == samples_.get()) {
    LOG_INFO << "Get the instance of sample, using T = "
        << T;
    samples_.reset(new Sample(T));
  } else {
    samples_->resize(T);
    LOG_WARNING << "Resize Sample OK!";
  }

  // LOG_WARNING << "Ready to configure Sample";
  sensors_->configureSample(*samples_);
  // LOG_WARNING << "Configure Sample SUCCESSFUL";
  if (nullptr != trial_controller_.get()) {
    LOG_INFO << "Trial Controller is configuring samples";
    trial_controller_->configureSample(*samples_);
    LOG_WARNING << "Trial Controller has configured samples";
  } else if (nullptr != reset_controller_.get()) {
    LOG_INFO << "Reset Controller is configuring samples";
    reset_controller_->configureSample(*samples_);
  } else {
    LOG_WARNING << "Configure the sample before instancing controller"
        << "(reset or trial controller), using the default value ("
        << "the number of joints).";
    // Set sample data format on the actions, which are not handled by any sensor.
    OptionsMap sample_metadata;
    samples_->setMetaData(gps::ACTION, U_.size(),
        SampleDataFormat::SampleDataFormatEigenVector, sample_metadata);
  }
}

void RobotPlugin2::configureLqr(
    const TrialCommand::ConstPtr& msg, CtrlConfigMap& params) {
  LinGaussParams lingauss = msg->controller.lingauss;
  int dX = (int) lingauss.dX;
  int dU = (int) lingauss.dU;
  // LOG_WARNING << "dX: " << dX << ", dU: " << dU;
  // Prepare options map
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

  trial_controller_.reset(new LQR);
  trial_controller_->configure(params);
}

inline void RobotPlugin2::configureCaffePolicy(
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
    LOG_ERROR << "NO DEFINE USE_CAFFE";
  }
}

inline void RobotPlugin2::configureTfPolicy(
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
    LOG_ERROR << "NO DEFINE USE_TF";
  }
}

// Data request callback.
void RobotPlugin2::reportSubCb(const DataRequest::ConstPtr& msg) {
  LOG(INFO) << ("received data request");
  is_report_waiting_ = true;
}

} /* namespace rl_agent */
