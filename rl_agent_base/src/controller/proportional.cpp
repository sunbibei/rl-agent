/*
Controller that executes a trial using a time-varying linear-Gaussian
control law.
*/

// Headers.
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>

#include "sample/sample.h"
#include "controller/proportional.h"

#include <foundation/utf.h>

namespace rl_agent {

// Constructor.
Propartional::Propartional(const std::string name)
  : TrialController() {
  is_configured_ = false;
}
// Destructor.
Propartional::~Propartional() { };

// Compute the action at the current time step.
void Propartional::getAction(int t,
    const Eigen::VectorXd& X, Eigen::VectorXd& U) {
  // Noise usually contained in k_
  if ((t < 0) || (t >= T_)) {
    LOG_WARNING << "Out of bound t (" << t << ") vs T (" << T_ << ")";
    return;
  }

  U = -K_[t] * (X - X_hat_);
  // LOG(WARNING) << "getAction: " << U;
}

void Propartional::configureSample(Sample& sample) {
  OptionsMap metadata;
  sample.setMetaData(gps::ACTION, K_[0].size(),
      SampleDataFormat::SampleDataFormatEigenVector, metadata);
}

// Configure the controller.
void Propartional::configure(CtrlConfigMap& options) {
  // Call superclass
  TrialController::configure(options);
  // TODO Don't do this hacky string indexing
  // LOG(WARNING) << "Set LQR parameters T: " << T_;
  K_.resize(T_);
  for(int i = 0; i < T_; i++) {
    K_[i] = boost::get<Eigen::MatrixXd>(
        options["K_" + std::to_string(i)]);
  }

  X_hat_ = boost::get<Eigen::VectorXd>(options["target"]);

  if (X_hat_.size() != K_[0].size()) {
    LOG_ERROR << "Some wrong happened between"
        << " the target position and K.";

    return;
  }

  state_defs_ = boost::get<std::vector<gps::SampleType>>(
      options["state_include"]);

  LOG_INFO << ("Set Propartional parameters SUCCESSFUL");
  is_configured_ = true;
}

} /* end namespace rl_agent */
