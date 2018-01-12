/*
 * chain_joint_encoder.h
 *
 *  Created on: Dec 26, 2016
 *      Author: silence
 */

#ifndef INCLUDE_RL_AGENT_SENSOR_CHAIN_JOINT_ENCODER_H_
#define INCLUDE_RL_AGENT_SENSOR_CHAIN_JOINT_ENCODER_H_

#include <Eigen/Dense>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include "sensor.h"
#include "sample/sample.h"
// #include "encoderfilter.h"

#include <repository/resource/joint.h>


namespace rl_agent {

class ChainJointEncoder: public Sensor {
public:
  ChainJointEncoder(const MiiString& name = "ChainJointEncoder");
  virtual bool init() override;

  virtual ~ChainJointEncoder();

public:
  virtual void update()                   override;
  virtual void configure(DataHandleMap&)  override;
  virtual void updateSample(Sample&, int) override;
  virtual void configureSample(Sample&)   override;

  ///! Helper methods
private:
  void initChainJointEncoder();
  bool initFKSolver();
  bool initJoints();
  void readFromRobot(Eigen::VectorXd& angles, Eigen::VectorXd&);
  // void configureVariants(DataHandleMap& handleMap);

private:
////////////////////////////////////////////////////////////////////////////////
/// About specially robot state variant
////////////////////////////////////////////////////////////////////////////////
  class Timer* timer_;
////////////////////////////////////////////////////////////////////////////////
/// About specially robot state variant
////////////////////////////////////////////////////////////////////////////////
  // KDL chains for the end-effectors.
  KDL::Chain fk_chain_;
  // KDL solvers for the end-effectors.
  boost::shared_ptr<KDL::ChainFkSolverPos> fk_solver_;
  // KDL solvers for end-effector Jacobians.
  boost::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  ///! The trained joint handles
  MiiVector<middleware::Joint*> res_map_;
////////////////////////////////////////////////////////////////////////////////
/// About calculate the position and Jacobian of end-effector
////////////////////////////////////////////////////////////////////////////////
  // Previous joint angles.
  Eigen::VectorXd previous_angles_;
  // Previous joint velocities.
  Eigen::VectorXd previous_velocities_;
  // Temporary storage for joint angles.
  Eigen::VectorXd temp_joint_angles_;
  // Temporary storage for KDL joint angle array.
  KDL::JntArray temp_joint_array_;
  // Temporary storage for KDL tip pose.
  KDL::Frame temp_tip_pose_;
  // Temporary storage for KDL Jacobian.
  KDL::Jacobian temp_jacobian_;
  // End-effector site/point offsets
  int n_points_;
  // EE Point jacobian
  Eigen::MatrixXd point_jacobians_;
  Eigen::MatrixXd point_jacobians_rot_;
  // boost::scoped_ptr<EncoderFilter> joint_filter_;

  // End-effector points in the space of the end-effector.
  Eigen::MatrixXd end_effector_points_;
  // Previous end-effector points.
  Eigen::MatrixXd previous_end_effector_points_;
  // End-effector points target.
  Eigen::MatrixXd end_effector_points_target_;
  // Velocities of points.
  Eigen::MatrixXd previous_end_effector_point_velocities_;
  // Temporary storage.
  Eigen::MatrixXd temp_end_effector_points_;
  // Previous end-effector position.
  Eigen::Vector3d previous_position_;
  // Previous end-effector rotation.
  Eigen::Matrix3d previous_rotation_;
  // Previous end-effector Jacobian.
  Eigen::MatrixXd previous_jacobian_;
};

} /* namespace rl_agent */

#endif /* INCLUDE_RL_AGENT_SENSOR_CHAIN_JOINT_ENCODER_H_ */
