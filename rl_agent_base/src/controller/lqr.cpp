
// Headers.

#include "controller/lqr.h"
#include "sample/sample.h"

namespace rl_agent {

LQR::LQR(const MiiString& name)
  : TrialController() {
  is_configured_ = false;
}

// Destructor.
LQR::~LQR() { };

// Compute the action at the current time step.
void LQR::getAction(int t,
    const Eigen::VectorXd& X, Eigen::VectorXd& U) {
  // Noise usually contained in k_
  if ((t < 0) || (t >= T_)) {
    LOG_WARNING << "Out of bound t (" << t << ") vs T (" << T_ << ")";
    return;
  }
  // LOG(WARNING) << "K_[t] of Size: " << K_[t].rows() << " * " << K_[t].cols();
  // LOG(WARNING) << "k_[t] of Size: = " << k_[t].rows() << " * " << k_[t].cols();

  // LOG(WARNING) << "X of Size: = " << X.rows() << " * " << X.cols();
  U = K_[t] * X + k_[t];
  // LOG(WARNING) << "getAction: " << U;
}

void LQR::configureSample(Sample& sample) {
  OptionsMap metadata;
  sample.setMetaData(gps::ACTION, k_[0].size(),
      SampleDataFormat::SampleDataFormatEigenVector, metadata);
}

// Configure the controller.
void LQR::configure(CtrlConfigMap& options) {
  // Call superclass
  TrialController::configure(options);
  // TODO Don't do this hacky string indexing
  // LOG(WARNING) << "Set LQR parameters T: " << T_;
  K_.resize(T_);
  for(int i = 0; i < T_; i++) {
    K_[i] = boost::get<Eigen::MatrixXd>(
        options["K_" + std::to_string(i)]);
  }

  k_.resize(T_);
  for(int i = 0; i < T_; i++) {
    k_[i] = boost::get<Eigen::VectorXd>(
        options["k_" + std::to_string(i)]);
  }

  state_defs_ = boost::get<std::vector<gps::SampleType>>(options["state_include"]);

  LOG_INFO << ("Set LQR parameters SUCCESSFUL");
  is_configured_ = true;
}

} /* end namespace rl_agent */
