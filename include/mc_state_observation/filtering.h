#pragma once
#include <SpaceVecAlg/SpaceVecAlg>
#include <boost/circular_buffer.hpp>
#include <gram_savitzky_golay/gram_savitzky_golay.h>

namespace filter
{

template<typename T>
class EigenVector
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  /** Filtering **/
  gram_sg::SavitzkyGolayFilterConfig sg_conf;
  gram_sg::SavitzkyGolayFilter sg_filter;
  // Buffers for Savitzky_golay
  boost::circular_buffer<T> buffer;

  // Outlier rejection
  T average_;
  T median_;

public:
  EigenVector(const gram_sg::SavitzkyGolayFilterConfig & conf)
  : sg_conf(conf), sg_filter(conf), buffer(2 * sg_filter.config().m + 1)
  { reset(T::Zero()); }

  void reset(const T & data)
  {
    average_ = data;
    median_ = data;
    // Initialize to data
    for(size_t i = 0; i < buffer.capacity(); i++) { buffer.push_back(data); }
  }

  void reset()
  {
    average_ = T::Zero();
    median_ = T::Zero();
    buffer.clear();
  }

  void compute_median(const T & data)
  {
    for(int i = 0; i < data.size(); ++i)
    {
      const auto & sample = data(i);
      average_(i) += (sample - average_(i)) * 0.1f; // rough running average.
      median_(i) += std::copysign(average_(i) * 0.01, sample - median_(i));
    }
  }
  void add(const T & data) { buffer.push_back(data); }
  T filter() const { return sg_filter.filter(buffer); }
  T median() const { return median_; }
  gram_sg::SavitzkyGolayFilterConfig config() const { return sg_conf; }

  bool ready() const { return buffer.size() == buffer.capacity(); }
};

/**
 * Rotation Filter
 * Based on Peter Cork lecture here:
 * https://www.cvl.isy.liu.se/education/graduate/geometry2010/lectures/Lecture7b.pdf
 * Adapted to real time filtering through Savitzky-Golay
 **/
class Rotation
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  /** Filtering **/
  gram_sg::SavitzkyGolayFilterConfig sg_conf;
  gram_sg::SavitzkyGolayFilter sg_filter;
  // Buffers for Savitzky_golay
  boost::circular_buffer<Eigen::Matrix3d> buffer;

public:
  Rotation(const gram_sg::SavitzkyGolayFilterConfig & conf);
  void reset(const Eigen::Matrix3d & r);
  void reset();
  void add(const Eigen::Matrix3d & r);
  Eigen::Matrix3d filter() const;
  gram_sg::SavitzkyGolayFilterConfig config() const { return sg_conf; }
  bool ready() const { return buffer.size() == buffer.capacity(); }
};

/**
 * @brief Filters PTransform
 * The transformations are first converted to their translation and RPY
 * compenents, and then each component is filtered individually
 * Finally the result is converted back to a PTransform
 */
class Transform
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  EigenVector<Eigen::Vector3d> trans_filter;
  Rotation rot_filter;

public:
  Transform(const gram_sg::SavitzkyGolayFilterConfig & conf);
  void reset(const sva::PTransformd & T);
  void reset();
  void add(const sva::PTransformd & T);
  sva::PTransformd filter() const;
  gram_sg::SavitzkyGolayFilterConfig config() const { return trans_filter.config(); }
  bool ready() const { return trans_filter.ready() && rot_filter.ready(); }
};

} // namespace filter
