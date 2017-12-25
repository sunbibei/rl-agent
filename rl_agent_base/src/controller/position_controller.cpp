/*
 * pid_controller.h
 *
 *  Created on: Dec 23, 2016
 *      Author: silence
 */

#include "controller/position_controller.h"
#include "sample/sample.h"

namespace rl_agent {

PositionController::PositionController(const std::string& name,
    double epspos, double epsvel)
    : Controller(), EPSILON_POS_(epspos), EPSILON_VEL_(epsvel)
{ }

PositionController::~PositionController() { }

// Check if controller is finished with its current task.
bool PositionController::isFinished() const {
  if (target_X_pos_.size() != last_X_pos_.size()) return false;
  // Check whether we are close enough to the current target.
  double error = (last_X_pos_ - target_X_pos_).norm();
  double vel = last_X_vel_.norm();
  return (error < EPSILON_POS_ && vel < EPSILON_VEL_);
}

void PositionController::reset() {
  p_ = Eigen::VectorXd();
  i_ = Eigen::VectorXd();
  d_ = Eigen::VectorXd();
  i_clamp_ = Eigen::VectorXd();
  pd_integral_ = Eigen::VectorXd();
  last_X_pos_ = Eigen::VectorXd();
  last_X_vel_ = Eigen::VectorXd();
  target_X_pos_ = Eigen::VectorXd();
  pos_error_ = Eigen::VectorXd();
}

// Configure the controller
void PositionController::configure(CtrlConfigMap& options) {
  // This sets the target position.
  // This sets the mode
  LOG_INFO << "Received controller configuration";
  Eigen::MatrixXd pd_gains = boost::get<Eigen::MatrixXd>(options["pd_gains"]);
  p_.resize(pd_gains.rows());
  i_.resize(pd_gains.rows());
  d_.resize(pd_gains.rows());
  i_clamp_.resize(pd_gains.rows());
  // LOG(WARNING) << "pd_gains.rows() = " << pd_gains.rows();
  for(int i = 0; i < pd_gains.rows(); ++i) {
    p_(i) = pd_gains(i, 0);
    i_(i) = pd_gains(i, 1);
    d_(i) = pd_gains(i, 2);
    i_clamp_(i) = pd_gains(i, 3);
  }
  target_X_pos_ = boost::get<Eigen::VectorXd>(options["data"]);
  state_defs_ = boost::get<std::vector<gps::SampleType>>(options["state_include"]);
}

void PositionController::configureSample(Sample& sample) {
  OptionsMap metadata;
  sample.setMetaData(gps::ACTION, target_X_pos_.size(),
      SampleDataFormat::SampleDataFormatEigenVector, metadata);
}

// Update the controller (take an action)
bool PositionController::control(const Eigen::VectorXd& X,
    Eigen::VectorXd& U) {

  if (timer_.is_running()) {
    timer_.start();
    return false;
  }
  // LOG(WARNING) << "Check dimensionality, X.rows = " << X.rows()
  //    << ", U.rows = " << U.rows() << ", last_X_pos_.rows" << last_X_pos_.rows();
  if (last_X_pos_.size() != X.size()) {
    // LOG(WARNING) << "Got the first X";
    last_X_pos_ = X; // Add new position;
    pd_integral_.resize(X.size());
    pd_integral_.fill(0.0);
  }

  // LOG(WARNING) << "X.rows = " << X.rows() << ", U.rows = "
  //        << U.rows() << ", last_X_pos_.rows" << last_X_pos_.rows();
  // Check dimensionality.
  assert(X.rows() == U.rows());
  assert(X.rows() == last_X_pos_.rows());

  double duration = timer_.dt_s();
  // Estimate joint angle velocities.
  if (0 != duration)
    last_X_vel_ = (X - last_X_pos_) / duration;

  last_X_pos_ = X;
  pos_error_ = X - target_X_pos_;
  pd_integral_ += pos_error_ * duration;

  for (int i = 0; i < pos_error_.rows(); i++){
    if (pd_integral_(i) * i_(i) > i_clamp_(i)) {
      pd_integral_(i) = i_clamp_(i) / i_(i);
    } else if (pd_integral_(i) * i_(i) < -i_clamp_(i)) {
      pd_integral_(i) = -i_clamp_(i) / i_(i);
    }
  }

  U = -((p_.array() * pos_error_.array()) +
        (d_.array() * last_X_vel_.array()) +
        (i_.array() * pd_integral_.array())).matrix();

  return true;
}

} /* namespace rl_agent */
