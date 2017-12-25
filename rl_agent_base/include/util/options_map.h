/*
The options object is used to list a map from strings to parameters.
*/

#ifndef INCLUDE_RL_AGENT_OPTIONS_MAP_H_
#define INCLUDE_RL_AGENT_OPTIONS_MAP_H_

// Headers.
#include <map>
#include <vector>
#include <string>
#include <boost/variant.hpp>
#include <eigen3/Eigen/Dense>

// Types of data supported for internal data storage.
enum OptionsDataFormat {
    OptionsDataFormatBool,
    OptionsDataFormatUInt8,
    OptionsDataFormatIntVector,
    OptionsDataFormatInt,
    OptionsDataFormatDouble,
    OptionsDataFormatMatrix,
    OptionsDataFormatVector,
    OptionsDataFormatString
};

// This is a parameter entry. Note that the arguments should match the enum.
typedef boost::variant<
    bool, uint8_t, std::vector<int>, int, double,
    Eigen::MatrixXd, Eigen::VectorXd, std::string
    > OptionsVariant;

// This is the options map.
typedef std::map<std::string, OptionsVariant> OptionsMap;

#endif /* INCLUDE_RL_AGENT_OPTIONS_MAP_H_ */
