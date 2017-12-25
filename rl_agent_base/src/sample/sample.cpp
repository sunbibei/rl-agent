/*
 * sample.cpp
 *
 *  Created on: Dec 21, 2016
 *      Author: silence
 */

#include "sample/sample.h"
#include <foundation/utf.h>

namespace rl_agent {

Sample::Sample(int T)
  : T_(T), t_(0) {
  LOG_INFO << "Initializing Sample with T = " << T;

  for (int i = 0; i < gps::TOTAL_DATA_TYPES; ++i) {
    TypeSample type_sample;// internal_data_[(gps::SampleType)i];
    type_sample.data_list_.resize(T_);
    type_sample.meta_data_.entries_size_ = 0;
    type_sample.meta_data_.data_format_  = SampleDataFormat::SampleDataFormatUnKnown;
    // internal_data_.insert(std::make_pair((gps::SampleType)i, type_sample));
    internal_data_[(gps::SampleType)i] = type_sample;
    // LOG(WARNING) << "Got " << i;
  }

  LOG_INFO << "Done Sample Constructor";
}

Sample::~Sample() { }

void Sample::resize(int T) {
  if ((T < 0) || T_ == T) return;
  LOG_INFO << "Initializing Sample with T = " << T;

  T_ = T;
  t_ = 0;
  for (auto& type : internal_data_) {
    type.second.data_list_.resize(T);
  }
}

void Sample::getMetaData(gps::SampleType type, MetaData& meta_data) {
  // while (!config_lock_.try_lock()) { }

  const MetaData& meta = internal_data_[type].meta_data_;
  meta_data.entries_size_       = meta.entries_size_;
  meta_data.data_format_        = meta.data_format_;
  meta_data.additional_info_    = meta.additional_info_;

  // config_lock_.unlock();
}

void Sample::setMetaData(gps::SampleType type, int rows, int cols,
    SampleDataFormat format, AdditionalInfo add_info) {
  // while (!config_lock_.try_lock()) { }

  TypeSample& type_sample                 = internal_data_[type];
  type_sample.meta_data_.entries_size_    = rows * cols;
  type_sample.meta_data_.data_format_     = format;
  type_sample.meta_data_.additional_info_ = add_info;
  // If this is a matrix or vector type, preallocate it now for fast copy later.
  switch (format) {
  case SampleDataFormat::SampleDataFormatEigenVector:
  {
    for (auto& data : type_sample.data_list_) {
      data = Eigen::VectorXd(rows);
    }
    break;
  }
  case SampleDataFormat::SampleDataFormatEigenMatrix:
  {
    for (auto& data : type_sample.data_list_) {
      data = Eigen::MatrixXd(rows, cols);
    }
    break;
  }
  default:
    break;
  }
  // config_lock_.unlock();
}

void Sample::setData(gps::SampleType type, SampleTypeVariant data,
                      int data_size, SampleDataFormat data_format, int t) {
  if ((t >= T_) or (t < 0)) {
    LOG_ERROR << "Out of bounds t(" << t << ") vs (" << T_ << ")" << "in setData";
    return;
  }
  // while (!config_lock_.try_lock()) { }

  TypeSample& type_sample = internal_data_[type];
  if (data_format != type_sample.meta_data_.data_format_) {
    LOG_ERROR << "Can't match sample format for the sample type: " << type;
    // config_lock_.unlock();
    return;
  }

  type_sample.data_list_[t] = data;
  // Only one entry, excepte vector and matrix
  // type_sample.meta_data_.entries_size_ = 1;
  t_ = t;
  // config_lock_.unlock();
}

/**
 * Setting data include EigenVector and EigenMatrix
 */
void Sample::setData(gps::SampleType type, double* data, int data_rows, int data_cols,
                      SampleDataFormat data_format, int t) {
  if ((t >= T_) or (t < 0)) {
    LOG_ERROR << "Out of bounds t(" << t << ") vs (" << T_ << ")" << " in setData"
        << ", The type of sample is " << type;
    return;
  }
  // while (!config_lock_.try_lock()) { }

  TypeSample& type_sample = internal_data_[type];
  if (data_format != type_sample.meta_data_.data_format_) {
    LOG_ERROR << "Can't match sample format for the sample type: " << type;
    // config_lock_.unlock();
    return;
  }

  switch (data_format) {
  case SampleDataFormat::SampleDataFormatEigenVector:
  {
    Eigen::VectorXd& vector = boost::get<Eigen::VectorXd>(type_sample.data_list_[t]);
    if (vector.rows() != data_rows or 1 != data_cols) {
      LOG_ERROR << "Invalid size in set_data! (" << vector.rows() << " vs " << data_rows
          << ") and cols (" << data_cols << ") for type SampleDataFormatEigenVector";
    }
    memcpy(vector.data(), data, sizeof(double) * data_rows * data_cols);
    break;
  }
  case SampleDataFormat::SampleDataFormatEigenMatrix:
  {
    Eigen::MatrixXd& matrix = boost::get<Eigen::MatrixXd>(type_sample.data_list_[t]);
    if ((matrix.rows() != data_rows) or (matrix.cols() != data_cols)) {
      LOG_ERROR << "Invalid size in set_data! (" << matrix.rows() << " vs " << data_rows
          << ") and (" << matrix.cols() << " vs " << data_cols
          << ") for type SampleDataFormatEigenMatrix";
    }
    memcpy(matrix.data(), data, sizeof(double) * data_rows * data_cols);
    break;
  }
  default:
    LOG_ERROR << "Cannot use set_data_vector with non-Eigen types! ";
  }

  // type_sample.meta_data_.entries_size_ = data_rows * data_cols;
  t_ = t;
  // config_lock_.unlock();
}

void Sample::getDataList(gps::SampleType datatype, Eigen::VectorXd &data, int T) {
  if ((T > T_) or (T < 0)) {
    LOG_ERROR << "Out of bounds t: " << T << "/" << T_ << " in getDataList";
    return;
  }

  // while (!config_lock_.try_lock());

  int size = internal_data_[datatype].meta_data_.entries_size_;
  // LOG(WARNING) << "getDataList :(" << datatype << ") in Sample: T = " << T << ", entries size = " << size;
  data.resize(size * T);
  std::vector<gps::SampleType> dtype_vector;
  dtype_vector.push_back(datatype);

  Eigen::VectorXd tmp_data;
  for(int t = 0; t < T; ++t) {
    getData(dtype_vector, tmp_data, t);
    // Fill in original data
    for(int i = 0; i < size; ++i) {
      data[t * size + i] = tmp_data[i];
    }
  }

  // config_lock_.unlock();
}

void Sample::getData(const std::vector<gps::SampleType>& datatypes, Eigen::VectorXd &data, int t) {
  if ((t >= T_) or (t < 0)) {
    LOG_ERROR << "Out of bounds t: " << t << "/" << T_ << " in getData";
    return;
  }
  // while (!config_lock_.try_lock()) { }

  // Calculate size
  //LOG(WARNING) << "Sample Get Data t: " << t;
  int total_size = 0;
  for (const auto& type : datatypes) {
    total_size += internal_data_[type].meta_data_.entries_size_;
    // LOG(WARNING) << "type: " << type << "'s size: " << internal_data_[type].meta_data_.entries_size_;
  }
  // LOG(WARNING) << "getData in Sample: t = " << t << ", total entries size = " << total_size;
  // LOG(WARNING) << "total size: " << total_size;
  data.resize(total_size);
  data.fill(0.0);

  // Fill in data
  int current_idx = 0;
  for(const auto& type: datatypes){
    const TypeSampleData&    sample_list    = internal_data_[type].data_list_;
    const MetaData&          sample_meta    = internal_data_[type].meta_data_;
    const SampleTypeVariant& sample_variant = sample_list[t];
    int size = sample_meta.entries_size_;
    //LOG(WARNING) << "size: " << size;
    //LOG(WARNING) << "format: " << (int)sample_meta.data_format_;
    // Handling for specific datatypes
    switch (sample_meta.data_format_) {
    case SampleDataFormat::SampleDataFormatEigenVector:
    {
      const Eigen::VectorXd& sensor_data = boost::get<Eigen::VectorXd>(sample_variant);
      data.segment(current_idx, size) = sensor_data;
      current_idx += size;
      break;
    }
    case SampleDataFormat::SampleDataFormatEigenMatrix:
    {
      Eigen::MatrixXd sensor_data = boost::get<Eigen::MatrixXd>(sample_variant).transpose();
            Eigen::VectorXd flattened_mat(Eigen::Map<Eigen::VectorXd>(sensor_data.data(), sensor_data.size()));
      flattened_mat.resize(sensor_data.cols()*sensor_data.rows(), 1);
      data.segment(current_idx, size) = flattened_mat;
      current_idx += size;
      break;
    }
    default:
      LOG_ERROR << "Datatypes currently must be in Eigen::Vector/Eigen::Matrix format. "
                  << "Offender: dtype=" << type;
      break;
    }
  }

  // config_lock_.unlock();
  return;
}

void Sample::getData(gps::SampleType type, Eigen::VectorXd& data, int t) {
  std::vector<gps::SampleType> tmp = {type};

  return getData(tmp, data, t);
}

void Sample::getShape(gps::SampleType type, std::vector<int>& shape) {
  // while (!config_lock_.try_lock()) { }

  const TypeSample& type_sample = internal_data_[type];
  int size = type_sample.meta_data_.entries_size_;
  shape.clear();

  switch (type_sample.meta_data_.data_format_) {
  case SampleDataFormat::SampleDataFormatEigenVector:
  {
    shape.push_back(size);
    break;
  }
  case SampleDataFormat::SampleDataFormatEigenMatrix:
  {
    // Grab shape from first entry at t = 0
    const Eigen::MatrixXd& data = boost::get<Eigen::MatrixXd>(type_sample.data_list_[0]);
    shape.push_back(data.rows());
    shape.push_back(data.cols());
    break;
  }
  default:
    // TODO Nothing to do ?
    break;
  }

  // config_lock_.unlock();
}

void Sample::getAvailableDataTypes(std::vector<gps::SampleType> &types) {
  types.clear();
  // while (!config_lock_.try_lock()) { }

  for (const auto& t : internal_data_) {
    if (SampleDataFormat::SampleDataFormatUnKnown
        != t.second.meta_data_.data_format_) {
      types.push_back(t.first);
    }
  }

  // config_lock_.unlock();
}

} /* namespace rl_agent */
