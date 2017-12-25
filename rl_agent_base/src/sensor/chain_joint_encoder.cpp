/*
 * chain_joint_encoder.h
 *
 *  Created on: Dec 26, 2016
 *      Author: silence
 */
#include "sensor/chain_joint_encoder.h"

#include <urdf/model.h>

#include <foundation/cfg_reader.h>
#include <repository/resource/joint_manager.h>

const auto INVALID_TIMEPOINT = std::chrono::high_resolution_clock::time_point::max();

namespace rl_agent {

struct Timer {
private:
  const std::chrono::high_resolution_clock::time_point INVALID_TIME_POINT;

public:
  ///! the first compute
  bool  first_compute_;
  ///! time control (in ms)
  int64_t dt_;
  std::chrono::high_resolution_clock::time_point curr_update_t_;
  std::chrono::high_resolution_clock::time_point last_update_t_;

  ///! these variables for debug.
  std::chrono::high_resolution_clock::time_point t0_;
  std::chrono::high_resolution_clock::time_point t1_;

  Timer()
    : INVALID_TIME_POINT(std::chrono::high_resolution_clock::time_point::max()),
      first_compute_(true), dt_(0),
      curr_update_t_(INVALID_TIME_POINT),
      last_update_t_(INVALID_TIME_POINT),
      t0_(INVALID_TIME_POINT), t1_(INVALID_TIME_POINT) {
    ;
  }

  bool running() {
    return (last_update_t_ != INVALID_TIME_POINT);
  }

  void start() {
    curr_update_t_ = std::chrono::high_resolution_clock::now();
    last_update_t_ = curr_update_t_;
    t0_            = curr_update_t_;
  }

  /*!
   * @brief The duration (in ms)
   */
  int64_t dt() {
    curr_update_t_ = std::chrono::high_resolution_clock::now();
    dt_            = std::chrono::duration_cast<std::chrono::milliseconds>
        (curr_update_t_ - last_update_t_).count();
    last_update_t_ = curr_update_t_;
    return dt_;
  }

  void stop(int64_t* span = nullptr) {
    t1_ = std::chrono::high_resolution_clock::now();
    curr_update_t_ = INVALID_TIME_POINT;
    last_update_t_ = INVALID_TIME_POINT;
    if (span) *span= std::chrono::duration_cast<std::chrono::milliseconds>
                        (t1_ - t0_).count();
  }
};


ChainJointEncoder::ChainJointEncoder(const std::string& name)
  : Sensor(name), timer_(new Timer), n_points_(1)
{ // setting type_include_
  type_include_.push_back(gps::JOINT_ANGLES);
  type_include_.push_back(gps::JOINT_VELOCITIES);
  type_include_.push_back(gps::END_EFFECTOR_POINTS);
  type_include_.push_back(gps::END_EFFECTOR_POINT_VELOCITIES);
  type_include_.push_back(gps::END_EFFECTOR_POINT_JACOBIANS);
  type_include_.push_back(gps::END_EFFECTOR_POINT_ROT_JACOBIANS);
  type_include_.push_back(gps::END_EFFECTOR_POSITIONS);
  type_include_.push_back(gps::END_EFFECTOR_JACOBIANS);
}

bool ChainJointEncoder::init() {
  if (!Sensor::init()) return false;
  LOG_INFO << "initialize sensor: " << sensor_name_;

  // LOG_INFO << "Configure the variants of sensor: " << sensor_name_;
  // configureVariants(handleMap);

  LOG_INFO << "Configure the joint encoder of sensor: " << sensor_name_;
  initChainJointEncoder();
  return true;
}

ChainJointEncoder::~ChainJointEncoder() {
  delete timer_;
}

void ChainJointEncoder::update() {
  if (!timer_->running()) {
    timer_->start();
    return;
  }

  double update_time = timer_->dt()/1000.0; // in s
  // Get new vector of joint angles from plugin.
  readFromRobot(temp_joint_angles_, previous_velocities_);
  // joint_filter_->update(update_time, temp_joint_angles_);
  // Get filtered joint angles
  // joint_filter_->get_state(temp_joint_angles_);
  // Compute end effector position, rotation, and Jacobian.
  // Save angles in KDL joint array.
  for (unsigned i = 0; i < temp_joint_angles_.size(); i++)
      temp_joint_array_(i) = temp_joint_angles_[i];
  // Run the solvers.
  fk_solver_->JntToCart(temp_joint_array_, temp_tip_pose_);
  jac_solver_->JntToJac(temp_joint_array_, temp_jacobian_);
  // Store position, rotation, and Jacobian.
  for (unsigned i = 0; i < 3; i++)
    previous_position_(i) = temp_tip_pose_.p(i);

  for (unsigned j = 0; j < 3; j++)
    for (unsigned i = 0; i < 3; i++)
      previous_rotation_(i,j) = temp_tip_pose_.M(i,j);
  for (unsigned j = 0; j < temp_jacobian_.columns(); j++)
    for (unsigned i = 0; i < 6; i++)
      previous_jacobian_(i,j) = temp_jacobian_(i,j);

  // IMPORTANT: note that the Python code will assume that the Jacobian is the Jacobian of the end effector points, not of the end
  // effector itself. In the old code, this correction was done in Matlab, but since the simulator will produce Jacobians of end
  // effector points directly, it would make sense to also do this transformation on the robot, and send back N Jacobians, one for
  // each feature point.
  // Compute jacobian
  // TODO - This assumes we are using all joints.
  unsigned n_actuator = previous_angles_.size();

  for(int i=0; i<n_points_; i++){
    unsigned site_start = i*3;
    Eigen::VectorXd ovec = end_effector_points_.col(i);

    for(unsigned j=0; j<3; j++){
      for(unsigned k=0; k<n_actuator; k++){
        point_jacobians_(site_start+j, k) = temp_jacobian_(j,k);
        point_jacobians_rot_(site_start+j, k) = temp_jacobian_(j+3,k);
      }
    }

    // Compute site Jacobian.
    ovec = previous_rotation_*ovec;
    for(unsigned k=0; k<n_actuator; k++){
      point_jacobians_(site_start  , k) += point_jacobians_rot_(site_start+1, k)*ovec[2] - point_jacobians_rot_(site_start+2, k)*ovec[1];
      point_jacobians_(site_start+1, k) += point_jacobians_rot_(site_start+2, k)*ovec[0] - point_jacobians_rot_(site_start  , k)*ovec[2];
      point_jacobians_(site_start+2, k) += point_jacobians_rot_(site_start  , k)*ovec[1] - point_jacobians_rot_(site_start+1, k)*ovec[0];
    }
  }

  // Compute current end effector points and store in temporary storage.
  temp_end_effector_points_ = previous_rotation_ * end_effector_points_;
  temp_end_effector_points_.colwise() += previous_position_;

  // Subtract the target end effector points so that the goal is always zero
  temp_end_effector_points_ -= end_effector_points_target_;

  // Compute velocities.
  previous_end_effector_point_velocities_ = (temp_end_effector_points_ - previous_end_effector_points_)/update_time;
  for (unsigned i = 0; i < previous_velocities_.size(); i++){
    previous_velocities_[i] = (temp_joint_angles_[i] - previous_angles_[i])/update_time;
  }

  // Move temporaries into the previous joint angles.
  previous_end_effector_points_ = temp_end_effector_points_;
  // std::cout << "POSITION: ";
  for (unsigned i = 0; i < previous_angles_.size(); i++) {
      previous_angles_[i] = temp_joint_angles_[i];
      // std::cout << previous_angles_[i] << " ";
  }
  // std::cout << std::endl;
}


void ChainJointEncoder::configure(DataHandleMap& opts) {
  auto ee_sites = opts.find("ee_sites");
  auto ee_pts_tgt = opts.find("ee_points_tgt");
  if (opts.end() == ee_sites || opts.end() == ee_pts_tgt) {
    LOG_INFO << "no end-effector sites or target!";
    return;
  }

  end_effector_points_ = boost::get<Eigen::MatrixXd>(ee_sites->second).transpose();
  n_points_ = end_effector_points_.cols();

  if( end_effector_points_.cols() != 3) {
      LOG_ERROR << "EE Sites have more than 3 coordinates: Shape=("
        << end_effector_points_.rows() << "," << end_effector_points_.cols() << ")";
  }

  end_effector_points_target_ = boost::get<Eigen::MatrixXd>(ee_pts_tgt->second).transpose();
  int n_points_target_ = end_effector_points_target_.cols();
  if( end_effector_points_target_.cols() != 3){
    LOG_ERROR << "EE tgt has more than 3 coordinates: Shape=("
        << end_effector_points_target_.rows() << "," << end_effector_points_target_.cols() << ")";
  }
  if(n_points_ != n_points_target_){
    LOG_ERROR << "Got " << n_points_target_ <<" ee_points_tgt (must match ee_points size: "
        << n_points_ << ")";
  }

  previous_end_effector_points_.resize(3, n_points_);
  previous_end_effector_point_velocities_.resize(3, n_points_);
  temp_end_effector_points_.resize(3, n_points_);
  point_jacobians_.resize(3 * n_points_, previous_angles_.size());
  point_jacobians_rot_.resize(3 * n_points_, previous_angles_.size());
}

void ChainJointEncoder::updateSample(Sample& sample, int t) {
  // Set joint angles.
  sample.setData(gps::JOINT_ANGLES, previous_angles_.data(),
      previous_angles_.size(), SampleDataFormat::SampleDataFormatEigenVector, t);
  // Set joint velocities.
  sample.setData(gps::JOINT_VELOCITIES, previous_velocities_.data(),
      previous_velocities_.size(), SampleDataFormat::SampleDataFormatEigenVector, t);
  // Set end effector point.
  sample.setData(gps::END_EFFECTOR_POINTS, previous_end_effector_points_.data(),
      previous_end_effector_points_.cols() * previous_end_effector_points_.rows(),
      SampleDataFormat::SampleDataFormatEigenVector, t);
  // Set end effector point velocities.
  sample.setData(gps::END_EFFECTOR_POINT_VELOCITIES,
      previous_end_effector_point_velocities_.data(),
      previous_end_effector_point_velocities_.cols() * previous_end_effector_point_velocities_.rows(),
      SampleDataFormat::SampleDataFormatEigenVector, t);
  // Set end effector point jacobian.
  sample.setData(gps::END_EFFECTOR_POINT_JACOBIANS,
      point_jacobians_.data(), point_jacobians_.rows(), point_jacobians_.cols(),
      SampleDataFormat::SampleDataFormatEigenMatrix, t);
  // Set end effector point rotation jacobian.
  sample.setData(gps::END_EFFECTOR_POINT_ROT_JACOBIANS,
      point_jacobians_rot_.data(), point_jacobians_rot_.rows(),
      point_jacobians_rot_.cols(), SampleDataFormat::SampleDataFormatEigenMatrix, t);
  // Set end effector position.
  sample.setData(gps::END_EFFECTOR_POSITIONS, previous_position_.data(), 3,
      SampleDataFormat::SampleDataFormatEigenVector, t);
  // Set end effector rotation.
  sample.setData(gps::END_EFFECTOR_ROTATIONS, previous_rotation_.data(), 3, 3,
      SampleDataFormat::SampleDataFormatEigenMatrix, t);
  // Set end effector jacobian.
  sample.setData(gps::END_EFFECTOR_JACOBIANS, previous_jacobian_.data(),
      previous_jacobian_.rows(), previous_jacobian_.cols(),
      SampleDataFormat::SampleDataFormatEigenMatrix, t);
}

void ChainJointEncoder::configureSample(Sample& sample) {
  LOG_INFO << "Configure Sample: " << sensor_name_;
  // Set joint angles size and format.
  OptionsMap metadata;
  sample.setMetaData(gps::JOINT_ANGLES, previous_angles_.size(),
      SampleDataFormat::SampleDataFormatEigenVector, metadata);
  // Set joint velocities size and format.
  sample.setMetaData(gps::JOINT_VELOCITIES, previous_velocities_.size(),
      SampleDataFormat::SampleDataFormatEigenVector, metadata);
  // Set end effector point size and format.
  sample.setMetaData(gps::END_EFFECTOR_POINTS, previous_end_effector_points_.cols()*previous_end_effector_points_.rows(),
      SampleDataFormat::SampleDataFormatEigenVector, metadata);
  // Set end effector point velocities size and format.
  sample.setMetaData(gps::END_EFFECTOR_POINT_VELOCITIES, previous_end_effector_point_velocities_.cols()*previous_end_effector_point_velocities_.rows(),
      SampleDataFormat::SampleDataFormatEigenVector, metadata);
  // Set end effector point jac size and format.
  sample.setMetaData(gps::END_EFFECTOR_POINT_JACOBIANS, point_jacobians_.rows(),point_jacobians_.cols(),
      SampleDataFormat::SampleDataFormatEigenMatrix, metadata);
  // Set end effector point jac size and format.
  sample.setMetaData(gps::END_EFFECTOR_POINT_ROT_JACOBIANS, point_jacobians_rot_.rows(),point_jacobians_rot_.cols(),
      SampleDataFormat::SampleDataFormatEigenMatrix, metadata);
  // Set end effector position size and format.
  sample.setMetaData(gps::END_EFFECTOR_POSITIONS,3,
      SampleDataFormat::SampleDataFormatEigenVector, metadata);
  // Set end effector rotation size and format.
  sample.setMetaData(gps::END_EFFECTOR_ROTATIONS,3,3,
      SampleDataFormat::SampleDataFormatEigenMatrix, metadata);
  // Set jacobian size and format.
  sample.setMetaData(gps::END_EFFECTOR_JACOBIANS, previous_jacobian_.rows(), previous_jacobian_.cols(),
      SampleDataFormat::SampleDataFormatEigenMatrix, metadata);
}

void ChainJointEncoder::initChainJointEncoder() {
  if (initFKSolver() && initJoints())
    LOG_INFO << "Initializing Chain joint encoder SUCCESSFUL!";
  else
    LOG_ERROR << "Initializing FKSolver or joints FAIL!";

  // Get current joint angles.
  // getJointEncoderReadings(previous_angles_);
  size_t n_jnts = res_map_.size();
  // Initialize previous angle.
  previous_angles_.resize(n_jnts);
  // Initialize velocities.
  previous_velocities_.resize(n_jnts);
  // Initialize temporary angles.
  temp_joint_angles_.resize(n_jnts);
  // Resize KDL joint array.
  temp_joint_array_.resize(n_jnts);
  // Resize Jacobian.
  previous_jacobian_.resize(6, n_jnts);
  temp_jacobian_.resize(n_jnts);

  // Allocate space for end effector points
  n_points_ = 1;
  previous_end_effector_points_.resize(3, 1);
  previous_end_effector_point_velocities_.resize(3, 1);
  temp_end_effector_points_.resize(3, 1);
  end_effector_points_.resize(3, 1);
  end_effector_points_.fill(0.0);
  end_effector_points_target_.resize(3, 1);
  end_effector_points_target_.fill(0.0);
  // Resize point jacobians
  point_jacobians_.resize(3, n_jnts);
  point_jacobians_rot_.resize(3, n_jnts);
  // Initialize and configure Kalman filter
  // joint_filter_.reset(new EncoderFilter(nh, previous_angles_));
  LOG_INFO << "Initialize ChainJointEncoder SUCCESSFUL!";
}

bool ChainJointEncoder::initFKSolver() {
  auto cfg = MiiCfgReader::instance();
  // Create FK solvers.
  // Get the name of the root.
  MiiString fk_tag = Label::make_label(getLabel(), "chain");
  MiiString root_name, tip_name, urdf_file;
  cfg->get_value_fatal(fk_tag, "root", root_name);
  cfg->get_value_fatal(fk_tag, "tip",  tip_name);
  cfg->get_value_fatal(fk_tag, "urdf", urdf_file);

  // KDL chains.
  urdf::Model robot_model;
  if (!robot_model.initFile(urdf_file)) {
    LOG_FATAL << "INIT robot model from file fail!";
    return false;
  }

  // Create active arm chain.
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(robot_model, kdl_tree)){
    LOG_ERROR << "Could not convert urdf into kdl tree";
    return false;
  }
  if (!kdl_tree.getChain(root_name, tip_name, fk_chain_)){
    LOG_FATAL << "Controller could not use the chain from '"
      << root_name << "' to '" << tip_name << "'";
    return false;
  }

  // Pose solvers.
  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(fk_chain_));
  jac_solver_.reset(new KDL::ChainJntToJacSolver(fk_chain_));
  return true;
}

bool ChainJointEncoder::initJoints() {
  auto cfg = MiiCfgReader::instance();
  MiiVector<MiiString> joint_names;
  cfg->get_value_fatal(getLabel(), "joints", joint_names);

  auto jnt_manager = middleware::JointManager::instance();
  // Pull out joint states.
  for (const auto& name : joint_names) {
    LOG_INFO << "Get joint handles: " << name;
    res_map_.push_back(jnt_manager->getJointHandle(name));
    // res_map_.insert(std::make_pair(name, robot_->getHandle(name)));
  }

  // Validate that the number of joints in the chain equals the length of the active arm joint state.
  LOG_INFO << "num_fk_chain: "    << fk_chain_.getNrOfJoints();
  LOG_INFO << "num_joint_state: " << res_map_.size();
  if (fk_chain_.getNrOfJoints() != res_map_.size()) {
      LOG_ERROR << "Number of joints in the active arm FK chain does not match "
          << "the number of joints in the active arm joint state!";
      return false;
  }

  return true;
}

void ChainJointEncoder::readFromRobot(Eigen::VectorXd& angles, Eigen::VectorXd& velocity) {
  if (angles.rows() != res_map_.size())
    angles.resize(res_map_.size());

  for (const auto& j : res_map_) {
    auto t = j->joint_type();
    angles(t)   = j->joint_position();
    velocity(t) = j->joint_velocity();
  }
}

//void ChainJointEncoder::configureVariants(DataHandleMap& handleMap) {
//  auto ee_sites   = handleMap.find("ee_sites");
//  auto ee_pts_tgt = handleMap.find("ee_points_tgt");
//  if (handleMap.end() == ee_sites
//      || handleMap.end() == ee_pts_tgt) {
//    LOG_INFO << "no end-effector sites or target!";
//    return;
//  }
//
//  end_effector_points_ = boost::get<Eigen::MatrixXd>(ee_sites->second).transpose();
//  n_points_ = end_effector_points_.cols();
//
//  if( end_effector_points_.cols() != 3) {
//      LOG_ERROR << "EE Sites have more than 3 coordinates: Shape=("
//        << end_effector_points_.rows() << "," << end_effector_points_.cols() << ")";
//  }
//
//  end_effector_points_target_ = boost::get<Eigen::MatrixXd>(ee_pts_tgt->second).transpose();
//  int n_points_target_ = end_effector_points_target_.cols();
//  if( end_effector_points_target_.cols() != 3){
//    LOG_ERROR << "EE tgt has more than 3 coordinates: Shape=("
//        << end_effector_points_target_.rows() << "," << end_effector_points_target_.cols() << ")";
//  }
//  if(n_points_ != n_points_target_){
//    LOG_ERROR << "Got " << n_points_target_ <<" ee_points_tgt (must match ee_points size: "
//        << n_points_ << ")";
//  }
//
//  previous_end_effector_points_.resize(3, n_points_);
//  previous_end_effector_point_velocities_.resize(3, n_points_);
//  temp_end_effector_points_.resize(3, n_points_);
//  point_jacobians_.resize(3 * n_points_, previous_angles_.size());
//  point_jacobians_rot_.resize(3 * n_points_, previous_angles_.size());
//}

} /* namespace rl_agent */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(rl_agent::ChainJointEncoder, Label)
CLASS_LOADER_REGISTER_CLASS(rl_agent::ChainJointEncoder, rl_agent::Sensor)
