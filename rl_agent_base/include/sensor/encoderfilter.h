/*
Kalman filter for encoder data. Each filter filters one degree of freedom at
a time.
*/
#ifndef _RL_AGENT_ENCODERFILTER_H_
#define _RL_AGENT_ENCODERFILTER_H_

#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

namespace rl_agent {

class EncoderFilter {
public:
  ///! Constructor.
  EncoderFilter(ros::NodeHandle& n, const Eigen::VectorXd &initial_state);
  ///! Destructor.
  virtual ~EncoderFilter();
  ///! Update the Kalman filter.
  virtual void update(double sec_elapsed, Eigen::VectorXd &state);
  ///! Configure the Kalman filter.
  virtual void configure(const std::string &params);
  ///! Return filtered state.
  virtual void get_state(Eigen::VectorXd &state) const;
  ///! Return filtered velocity.
  virtual void get_velocity(Eigen::VectorXd &velocity) const;

private:
  Eigen::MatrixXd time_matrix_;
  Eigen::MatrixXd filtered_state_;
  Eigen::VectorXd observation_vector_;

  int  num_joints_;
  bool is_configured_;
  void split(const std::string& s, char delim, std::vector<std::string>& elems);

};

} /* namespace rl_agent */

#endif
