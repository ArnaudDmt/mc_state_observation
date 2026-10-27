#include <mc_state_observation/filtering.h>

// Filtering
#include <boost/circular_buffer.hpp>
#include <Eigen/SVD>
#include <gram_savitzky_golay/gram_savitzky_golay.h>

namespace filter
{

Rotation::Rotation(const gram_sg::SavitzkyGolayFilterConfig & conf)
: sg_conf(conf), sg_filter(conf), buffer(2 * sg_filter.config().m + 1)
{ reset(Eigen::Matrix3d::Zero()); }

void Rotation::reset(const Eigen::Matrix3d & r)
{
  // Initialize to data
  for(size_t i = 0; i < buffer.capacity(); i++) { buffer.push_back(r); }
}

void Rotation::reset()
{ buffer.clear(); }

void Rotation::add(const Eigen::Matrix3d & r)
{ buffer.push_back(r); }

Eigen::Matrix3d Rotation::filter() const
{
  // Apply a temporal (savitzky-golay) convolution,
  // followed by an orthogonalization
  const Eigen::Matrix3d & result = sg_filter.filter(buffer);
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(result, Eigen::ComputeFullV | Eigen::ComputeFullU);
  Eigen::Matrix3d res = svd.matrixU() * svd.matrixV().transpose();
  return res;
}

Transform::Transform(const gram_sg::SavitzkyGolayFilterConfig & conf) : trans_filter(conf), rot_filter(conf) {}

void Transform::reset(const sva::PTransformd & T)
{
  trans_filter.reset(T.translation());
  rot_filter.reset(T.rotation());
}

void Transform::reset()
{
  trans_filter.reset();
  rot_filter.reset();
}

void Transform::add(const sva::PTransformd & T)
{
  trans_filter.add(T.translation());
  rot_filter.add(T.rotation());
}

sva::PTransformd Transform::filter() const
{
  const Eigen::Vector3d & trans_res = trans_filter.filter();
  const Eigen::Matrix3d & rot_res = rot_filter.filter();
  return sva::PTransformd(rot_res, trans_res);
}

} // namespace filter
