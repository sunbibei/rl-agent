/*
 * pid_controller.h
 *
 *  Created on: Dec 23, 2016
 *      Author: silence
 */

#ifndef INCLUDE_RL_AGENT_CONTROLLER_POSITION_CONTROLLER_H_
#define INCLUDE_RL_AGENT_CONTROLLER_POSITION_CONTROLLER_H_

#include "controller.h"
#include <toolbox/timer.h>

namespace rl_agent {

class PositionController: public Controller {
public:
  PositionController(const std::string& name = "PID",
      double epspos = 0.185, double epsvel = 0.01);
  virtual ~PositionController();

  // Check if controller is finished with its current task.
  virtual bool isFinished() const override;
  virtual void reset();
  // Configure the controller
  virtual void configure(CtrlConfigMap& options) override;
  virtual void configureSample(Sample& sample)   override;
  // Update the controller (take an action)
  virtual bool control(const Eigen::VectorXd& X,
      Eigen::VectorXd& U) override;

private:
  // P I D gains.
  Eigen::VectorXd p_;
  Eigen::VectorXd i_;
  Eigen::VectorXd d_;
  Eigen::VectorXd i_clamp_;
  Eigen::VectorXd pd_integral_;
  Eigen::VectorXd last_X_pos_;
  Eigen::VectorXd last_X_vel_;
  Eigen::VectorXd target_X_pos_;
  Eigen::VectorXd pos_error_;

  const double EPSILON_POS_;
  const double EPSILON_VEL_;

  middleware::Timer timer_;
};

} /* namespace rl_agent */

#endif /* INCLUDE_RL_AGENT_CONTROLLER_POSITION_CONTROLLER_H_ */
