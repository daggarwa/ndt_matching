#ifndef NDT_MATCHING__NDT_LIB_HPP_
#define NDT_MATCHING__NDT_LIB_HPP_

#include "ndt_matching/visibility_control.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace ndt_matching {

struct Pose {
  Eigen::Affine3f transform;
};

class NdtLib {
 private:
  pcl::VoxelGridCovariance<pcl::PointXYZ> m_voxelGrid;
  float m_resolution;

  size_t m_maxIterations;
  float m_outlierRatio;
  float m_Epsilon;

  Pose m_poseEstimate;

  void computeJacobian(Pose const& poseGuess,
                       Eigen::Matrix<float, 3, 6>& Jacobian3D);
  void computeHessian(Pose const& poseGuess,
                      Eigen::Matrix<float, 18, 6>& Hessian3D);

 public:
  NdtLib();
  void setMap(pcl::PointCloud<pcl::PointXYZ>::Ptr map, float resolution);
  void setParams(size_t maxIter, float epsilon, float outlierRatio);
  Pose run(pcl::PointCloud<pcl::PointXYZ> const& bagCloud,
           Pose const& poseGuess);
  Pose getCurrentPoseEstimate();
  virtual ~NdtLib();
};

}  // namespace ndt_matching

#endif  // NDT_MATCHING__NDT_LIB_HPP_
