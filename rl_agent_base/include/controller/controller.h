/*
 * controller.h
 *
 *  Created on: Dec 21, 2016
 *      Author: silence
 */

#ifndef INCLUDE_RL_AGENT_CONTROLLER_H_
#define INCLUDE_RL_AGENT_CONTROLLER_H_

#include <boost/scoped_ptr.hpp>
#include <boost/variant.hpp>

#include <Eigen/Dense>

#include "gps/proto/gps.pb.h"

#include <foundation/utf.h>
#include <foundation/label.h>

namespace rl_agent {

// This is a parameter entry. Note that the arguments should match the enum.
typedef boost::variant<bool, int, int64_t, double, std::string,
      std::vector<std::string>, Eigen::MatrixXd, Eigen::VectorXd,
      std::vector<gps::SampleType>> CtrlConfigVar;

// This is the options map.
typedef std::map<std::string, CtrlConfigVar> CtrlConfigMap;


class Sample;

class Controller {
public:
  ///! Constructor
  // Controller() { };
  ///! Deconstructor
  virtual ~Controller() { };

  const std::vector<gps::SampleType>& state_types()
      { return state_defs_; }

public:
  // Member Methods
  // Update the controller (take an action)
  virtual bool control(const Eigen::VectorXd& X, Eigen::VectorXd& U) = 0;
  // Check if controller is finished with its current task.
  virtual bool isFinished() const       = 0;
  virtual void configureSample(Sample&) = 0;

  // Reset the controller -- this is typically called when the controller is turned on.
  virtual void reset() { };
  // Configure the controller
  virtual void configure(CtrlConfigMap& options) { };

protected:
  MiiVector<gps::SampleType> state_defs_;
};

} /* namespace rl_agent */

#endif /* INCLUDE_RL_AGENT_CONTROLLER_H_ */
