/*
Controller that executes a trial using a time-varying linear-Gaussian
control law.
*/
#ifndef INCLUDE_RL_AGENT_LQR_H_
#define INCLUDE_RL_AGENT_LQR_H_

// Headers.
#include "trial_controller.h"
// Superclass.

namespace rl_agent {

class LQR : public TrialController {
public:
  // Constructor.
  LQR(const MiiString& name = "LQR");
  // Destructor.
  virtual ~LQR();

public:
  // Compute the action at the current time step.
  virtual void getAction(int t,
      const Eigen::VectorXd& X, Eigen::VectorXd& U) override;

  virtual void configureSample(Sample& sample)      override;
  // Configure the controller.
  virtual void configure(CtrlConfigMap& options) override;

private:
  // Linear feedbacks.
  std::vector<Eigen::MatrixXd> K_;
  // Bias.
  std::vector<Eigen::VectorXd> k_;
};

} /* end namespace rl_agent */

#endif
