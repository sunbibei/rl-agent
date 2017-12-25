/*
The state object maintains the state, assembles state and observation vectors,
and keeps track of what is and is not included in the state. This object is
used both by the controller, to incrementally assemble the state during a
trial, to keep track of sample data and get the state and observation vectors
from it.
*/

#ifndef INCLUDE_RL_AGENT_SAMPLE_H_
#define INCLUDE_RL_AGENT_SAMPLE_H_

#include <map>
#include <mutex>
#include <vector>
#include <boost/variant.hpp>
#include <Eigen/Dense>

#include "gps/proto/gps.pb.h"
#include "util/options_map.h"

namespace rl_agent {

// Types of data supported for internal data storage.
enum class SampleDataFormat {
  SampleDataFormatUnKnown = -1, // Error flag
  SampleDataFormatBool,         // bool
  SampleDataFormatUInt8,        // uint8_t
  SampleDataFormatUInt16,       // ??? TODO can't match the SampleVariant
  SampleDataFormatInt,          // int
  SampleDataFormatDouble,       // double
  SampleDataFormatEigenMatrix,  // Eigen::MatrixXd
  SampleDataFormatEigenVector   // Eigen::VectorXd
};

// typedef SampleMap
typedef boost::variant<
    bool,   uint8_t,         std::vector<int>, int,
    double, Eigen::MatrixXd, Eigen::VectorXd
    > SampleTypeVariant;
// Each element is one type in the sample each time-step
typedef std::vector<SampleTypeVariant> TypeSampleData;

typedef OptionsMap AdditionalInfo;
/**
 * MetaData is data that provides information about other data
 */
typedef struct {
  // format of each field
  SampleDataFormat  data_format_;
  // size of each field (in number of entries, not bytes)
  int               entries_size_;
  // additional information about each field.
  AdditionalInfo    additional_info_;
} MetaData;

/**
 * The data associated with one sample type and meta data
 */
typedef struct {
  TypeSampleData data_list_;
  MetaData       meta_data_;
} TypeSample;
typedef std::map<gps::SampleType, TypeSample> SampleMap;

class Sample {
public:
  // Constructor and Destructor
  Sample(int T);
  virtual ~Sample();

  virtual void resize(int T);
  // Get/Set meta-data
  virtual void getMetaData(gps::SampleType, MetaData&);
  virtual void setMetaData(gps::SampleType type, int data_rows, int data_cols,
      SampleDataFormat format, AdditionalInfo add_info);

  // Get pointer to internal data for given time step
  virtual void* getDataPointer(int, gps::SampleType) { return nullptr; };

  // Add sensor data for given timestep
  virtual void setData(gps::SampleType type, SampleTypeVariant data, int data_size,
                        SampleDataFormat data_format, int t);
  virtual void setData(gps::SampleType type, double* data, int data_rows, int data_cols,
                        SampleDataFormat data_format, int t);

  virtual void getDataList(gps::SampleType datatype, Eigen::VectorXd &data, int T);
  virtual void getData(const std::vector<gps::SampleType>& datatypes, Eigen::VectorXd& data, int t);
  virtual void getData(gps::SampleType datatypes, Eigen::VectorXd& data, int t);

  // Get shape with dimensions of the specify sample type
  void getShape(gps::SampleType sample_type, std::vector<int>& shape);
  // Get datatypes which have metadata set
  void getAvailableDataTypes(std::vector<gps::SampleType> &types);

  // Get the T/state/action
  int getT() const { return T_; };

  // Only in order to offer more convenient interface
  void setMetaData(gps::SampleType type, int rows,
          SampleDataFormat format, AdditionalInfo add_info) {
    setMetaData(type, rows, 1, format, add_info);
  }
  void setMetaData(gps::SampleType type, MetaData meta_data) {
    setMetaData(type, meta_data.entries_size_, 1,
          meta_data.data_format_, meta_data.additional_info_);
  }
  void setData(gps::SampleType type, double* data,
                int data_size, SampleDataFormat data_format, int t) {
    setData(type, data, data_size, 1, data_format, t);
  }

private:
  // Length of sample
  int T_;
  // the current time step
  int t_;
  // sensor data for all time steps.
  SampleMap internal_data_;
  // mutex for configure
  // std::mutex config_lock_;
};

} /* namespace rl_agent */

#endif /* INCLUDE_RL_AGENT_SAMPLE_H_ */
