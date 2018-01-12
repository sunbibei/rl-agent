/*
Kalman filter for encoder data. Each filter filters one degree of freedom at
a time.
*/

#include "sensor/encoderfilter.h"
#include <foundation/utf.h>

namespace rl_agent {

void EncoderFilter::split(const std::string& s, char delim, std::vector<std::string>& elems) {
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
}

// Constructor.
EncoderFilter::EncoderFilter(ros::NodeHandle& n, const Eigen::VectorXd &initial_state) {
  // Set initial state.
  num_joints_ = initial_state.size();

  is_configured_ = false;
  std::string params;
  if (!n.getParam("encoder_filter_params", params)) {
      LOG_ERROR << "Failed to receive joint kalman filter params "
          << "in namespace " << n.getNamespace();
      return;
  }

  configure(params);

  filtered_state_.resize(1, num_joints_);
  for (int i = 0; i < num_joints_; ++i) {
      filtered_state_(0,i) = initial_state(i);
  }

  LOG_INFO << "EncoderFilter Constructor call SUCCESSFUL!";
}
// Destructor.
EncoderFilter::~EncoderFilter() { }
// Update the Kalman filter.
void EncoderFilter::update(double sec_elapsed, Eigen::VectorXd &state) {
  if (is_configured_)
    filtered_state_ = time_matrix_ * filtered_state_ + observation_vector_ * state.transpose();
  else
    LOG_ERROR << ("Not implemented if not configured");
}

// Configure the Kalman filter.
void EncoderFilter::configure(const std::string &params) {
  LOG_INFO << ("Configuring encoder Kalman filter.");
  std::vector<std::string> matrices;
  split(params, ';', matrices);
  LOG_INFO << ("Split encoder Kalman filter parameter by ';'.");
  // First line is time matrix
  std::vector<std::string> time_values;
  split(matrices[0], ' ', time_values);
  // Second line is observation vector
  std::vector<std::string> obs_values;
  split(matrices[1], ' ', obs_values);

  LOG_INFO << ("Split encoder Kalman filter parameter by ' '.");
  int filter_order = obs_values.size();
  time_matrix_.resize(filter_order, filter_order);
  observation_vector_.resize(filter_order);

  filtered_state_.resize(filter_order, num_joints_);
  filtered_state_.fill(0.0);

  LOG_INFO << ("Resize and fill");
  for (int i = 0; i < filter_order; ++i) {
    for (int j = 0; j < filter_order; ++j) {
      time_matrix_(i,j) = (double) atof(time_values[i + j*filter_order].c_str());
    }
    observation_vector_(i) = (double) atof(obs_values[i].c_str());
  }
  is_configured_ = true;
  LOG_INFO << ("Joint kalman filter configured.");
}

// Return filtered state.
void EncoderFilter::get_state(Eigen::VectorXd &state) const {
  state = filtered_state_.row(0);
}
// Return filtered velocity.
void EncoderFilter::get_velocity(Eigen::VectorXd &velocity) const {
  velocity = filtered_state_.row(1);
}

} /* namespace rl_agent */
