/*
 * composite_mode.h
 *
 *  Created on: Dec 22, 2016
 *      Author: silence
 */

#ifndef INCLUDE_RL_AGENT_UTIL_COMPOSITE_H_
#define INCLUDE_RL_AGENT_UTIL_COMPOSITE_H_

#include <string>
#include <map>

#include <boost/shared_ptr.hpp>
#include <glog/logging.h>

namespace rl_agent {

template <typename T>
class Composite {
public:
  Composite(const std::string& name) : name_(name) { };
  virtual ~Composite() { };

  const std::string& getName() {return name_;}
  void setName(const std::string& name) {name_ = name;}

  virtual void add(T* component) {
    auto itr = composite_.find(component->getName());
    if (composite_.end() == itr) {
      LOG(INFO) << "Addition component( " << component->getName() << " )";
      boost::shared_ptr<T> ptr(component);
      composite_.insert(std::make_pair(component->getName(), ptr));
    } else {
      LOG(WARNING) << "Replace component( " << component->getName() << " )";
      itr->second.reset(component);
    }
  }

  virtual void add(boost::shared_ptr<T> component) {
    auto itr = composite_.find(component->getName());
    if (composite_.end() == itr) {
      LOG(INFO) << "Addition component( " << component->getName() << " )";
      composite_.insert(std::make_pair(component->getName(), component));
    } else {
      LOG(WARNING) << "Replace component( " << component->getName() << " )";
      itr->second.swap(component);
    }
  }

  virtual void remove(T* component) {
    remove(component->getName());
  }

  virtual void remove(const std::string& name) {
    auto itr = composite_.find(name);
    if (composite_.end() != itr) {
      composite_.erase(itr);
    }
    LOG(INFO) << "Remove component( " << name << " )";
  }

protected:
  std::string name_;
  std::map<std::string, boost::shared_ptr<T>> composite_;
};

} /* namespace rl_agent */

#endif /* INCLUDE_RL_AGENT_UTIL_COMPOSITE_H_ */
