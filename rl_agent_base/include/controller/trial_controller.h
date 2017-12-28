/*
Controller that executes a trial, using a control strategy that is defined in
a subclass.
*/
#ifndef INCLUDE_TRIAL_CONTROLLER_H_
#define INCLUDE_TRIAL_CONTROLLER_H_

// Superclass.
#include "controller.h"
#include <toolbox/time_control.h>

namespace rl_agent {

class TrialController : public Controller {
public:
  // Constructor.
  TrialController(/*size_t T, int64_t dt*/);
  // Destructor.
  virtual ~TrialController();

  // Compute the action at the current time step.
  virtual void getAction(int t,
      const Eigen::VectorXd& state, Eigen::VectorXd& action) = 0;

  // Update the controller (take an action).
  virtual bool control(const Eigen::VectorXd& state,
      Eigen::VectorXd &action) override;

  // Configure the controller.
  virtual void configure(CtrlConfigMap& options) override;
  virtual bool isFinished()   const;
  virtual bool isConfigured() const;

  // Return trial step index
  int getStepCounter();
  // Return length of trial.
  int getTrialLength();

protected:
  // Holds the step length of a trial
  int T_;
  // Holds the current step of a trial
  int t_;

  bool is_configured_;

  TimeControl timer_;
  // the tmp duration
  int64_t   tmp_duration_;
  // Store the controller step duration (in ms)
  int64_t   step_duration_;
};

} /* end namespace rl_agent */

#endif
