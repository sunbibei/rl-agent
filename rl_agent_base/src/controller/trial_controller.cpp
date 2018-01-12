
// Headers.
#include <vector>
#include <eigen3/Eigen/Dense>
#include <boost/scoped_ptr.hpp>

#include "gps/proto/gps.pb.h"

// Superclass.
#include "controller/trial_controller.h"
#include <foundation/utf.h>

namespace rl_agent {

// Constructor.
TrialController::TrialController(/*size_t T, int64_t dt*/)
  : T_(0), t_(0), is_configured_(false),
    tmp_duration_(0), step_duration_(0) {
  // Set initial time.
}
// Destructor.
TrialController::~TrialController() { };

// Update the controller (take an action).
bool TrialController::control(const Eigen::VectorXd& state,
    Eigen::VectorXd &action) {
  if (!is_configured_)      return false;
  if (!timer_.running()) timer_.start();

  tmp_duration_ += timer_.dt();
  if ((isFinished()) || (tmp_duration_ < step_duration_)) return false;

  getAction(t_, state, action);
  LOG_INFO << "Step counter: " << t_;

  ++t_;
  tmp_duration_ = 0;
  return true;
}

// Configure the controller.
void TrialController::configure(CtrlConfigMap& options) {
  LOG_INFO << ">TrialController::configure controller";
  if(!isFinished()){
    LOG_WARNING << ("Cannot configure controller while a trial is in progress");
  }
  T_ = boost::get<int>(options["T"]);
  t_ = 0;
  step_duration_ = boost::get<int64_t>(options["ctrl_frequency"]);
  is_configured_ = true;
}

bool TrialController::isFinished()   const { return t_ >= T_; }
bool TrialController::isConfigured() const { return is_configured_; }

// Return trial step index
int TrialController::getStepCounter() { return t_; }
// Return length of trial.
int TrialController::getTrialLength() { return T_; }

} /* end namespace rl_agent */
