/*
Controller that executes a trial using a time-varying linear-Gaussian
control law.
*/
#pragma once

// Headers.
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>

#include "sample/sample.h"
#include "trial_controller.h"
// Superclass.

namespace rl_agent {

class Propartional : public TrialController {
public:
  // Constructor.
  Propartional(const std::string name = "Propartional");
  // Destructor.
  virtual ~Propartional();
  // Compute the action at the current time step.
  virtual void getAction(int t,
      const Eigen::VectorXd& X, Eigen::VectorXd& U) override;
  virtual void configureSample(Sample& sample) override;
  // Configure the controller.
  virtual void configure(CtrlConfigMap& options) override;

private:
  // Linear feedbacks.
  std::vector<Eigen::MatrixXd> K_;
  // The Target Position
  Eigen::VectorXd X_hat_;

};

}
