#include "ndt_matching/ndt_lib.hpp"

namespace ndt_matching {

NdtLib::NdtLib() {}

// Function to register given bag cloud and output estimated pose
Pose NdtLib::run(pcl::PointCloud<pcl::PointXYZ> const& bagCloud,
                 Pose const& poseGuess) {
  m_poseEstimate = poseGuess;
  bool converged = false;
  size_t iterations = 0;
  Eigen::Matrix<float, 3, 6> Jacobian3D;
  Eigen::Matrix<float, 18, 6> Hessian3D;
  Eigen::Matrix<float, 6, 1> g;
  Eigen::Matrix<float, 6, 6> H;
  float score = 0.0f;

  pcl::PointCloud<pcl::PointXYZ> transformedCloud = bagCloud;

  Eigen::Matrix<float, 6, 1> p, delta_p;
  // Initialize current pose estimate p
  pcl::getTranslationAndEulerAngles(m_poseEstimate.transform, p[0], p[1], p[2],
                                    p[3], p[4], p[5]);

  // Constants for estimating gaussion distribution from Eq (6.7) and
  // Eq(6.8)
  float c1, c2, d1, d2, d3;
  c1 = 10 * (1 - m_outlierRatio);
  c2 = m_outlierRatio / pow(m_resolution, 3);
  d3 = -log(c2);
  d1 = -log(c1 + c2) - d3;
  d2 = -2 * log((-log(c1 * exp(-0.5) + c2) - d3) / d1);

  while (!converged) {
    Jacobian3D.setZero();
    Hessian3D.setZero();
    g.setZero();
    H.setZero();
    score = 0.0;
    // Calculate T(p,xk) for all points in the cloud
    transformPointCloud(bagCloud, transformedCloud, m_poseEstimate.transform);
    for (size_t i = 0; i < transformedCloud.points.size(); i++) {
      pcl::PointXYZ pointInCloud = transformedCloud.points[i];
      auto xk = Eigen::Vector3f(pointInCloud.x, pointInCloud.y, pointInCloud.z);

      pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf const* voxel =
          m_voxelGrid.getLeaf(xk);

      if (!voxel) continue;
      // Calculate p(xk) from Eq(6.9)
      Eigen::Vector3f xkMinusMean = xk - voxel->getMean().cast<float>();
      Eigen::Matrix3f sigmakInverse = voxel->getInverseCov().cast<float>();
      auto gaussian =
          exp(-d2 * xkMinusMean.dot(sigmakInverse * xkMinusMean) / 2);
      float p_x = -d1 * gaussian;

      // Score is summation over all xk from Eq (6.10)
      score += p_x;

      float d2MulGaussian = d2 * gaussian;
      // Error checking for invalid values.
      if (d2MulGaussian > 1 || d2MulGaussian < 0) continue;

      // Portion of Equation 6.12 and 6.13
      float d1d2MulGaussian = d2MulGaussian * d1;
      // Portion of Equation 6.12 and 6.13
      computeJacobian(m_poseEstimate, Jacobian3D);
      computeHessian(m_poseEstimate, Hessian3D);

      for (int i = 0; i < 6; i++) {
        // Portion of Equation 6.12 and 6.13
        auto sigmakInverseMulJi = sigmakInverse * Jacobian3D.col(i);

        // Update gradient from Equation 6.12
        g(i) += xkMinusMean.dot(sigmakInverseMulJi) * d1d2MulGaussian;

        for (int j = 0; j < H.cols(); j++) {
          // Update hessian from Equation 6.13
          H(i, j) += d1d2MulGaussian *
                     (-d2 * xkMinusMean.dot(sigmakInverseMulJi) *
                          xkMinusMean.dot(sigmakInverse * Jacobian3D.col(j)) +
                      xkMinusMean.dot(sigmakInverse *
                                      Hessian3D.block<3, 1>(3 * i, j)) +
                      Jacobian3D.col(j).dot(sigmakInverseMulJi));
        }
      }
    }

    // Solve H*delta_p = -g for delta_p
    Eigen::JacobiSVD<Eigen::Matrix<float, 6, 6> > sv(
        H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    delta_p = sv.solve(-g);

    p = p + delta_p;
    // Calculate norm to be used in convergence check
    auto norm = delta_p.norm();
    iterations++;

    if (iterations >= m_maxIterations || (norm <= m_Epsilon)) {
      converged = true;
    }

    pcl::getTransformation(p[0], p[1], p[2], p[3], p[4], p[5],
                           m_poseEstimate.transform);
  }

  return m_poseEstimate;
}

void NdtLib::computeJacobian(Pose const& po,
                             Eigen::Matrix<float, 3, 6>& Jacobian3D) {
  float x1, x2, x3;
  float roll, pitch, yaw;

  pcl::getTranslationAndEulerAngles(po.transform, x1, x2, x3, roll, pitch, yaw);
  float cx, cy, cz, sx, sy, sz;
  cx = cos(roll);
  sx = sin(roll);
  cy = cos(pitch);
  sy = sin(pitch);
  cz = cos(pitch);
  sz = sin(pitch);

  // From Eq (6.19)
  auto a = x1 * (-sx * sz + cx * sy * cz) + x2 * (-sx * cz - cx * sy * sz) +
           x3 * (-cx * cy);
  auto b = x1 * (cx * sz + sx * sy * cz) + x2 * (cx * cz - sx * sy * sz) +
           x3 * (-sx * cy);
  auto c = x1 * (-sy * cz) + x2 * sy * sz + x3 * cy;
  auto d = x1 * sx * cy * cz + x2 * (-sx * cy * sz) + x3 * sx * sy;
  auto e = x1 * (-cx * cy * cz) + x2 * cx * cy * sz + x3 * (-cx * sy);
  auto f = x1 * (-cy * sz) + x2 * (-cy * cz) + x3 * 0;
  auto g =
      x1 * (cx * cz - sx * sy * sz) + x2 * (-cx * sz - sx * sy * cz) + x3 * 0;
  auto h =
      x1 * (sx * cz + cx * sy * sz) + x2 * (cx * sy * cz - sx * sz) + x3 * 0;

  // From Eq (6.18)
  Jacobian3D << 1, 0, 0, 0, c, f, 0, 1, 0, a, d, g, 0, 0, 1, b, e, h;
}

void NdtLib::computeHessian(Pose const& po,
                            Eigen::Matrix<float, 18, 6>& Hessian3D) {
  float x1, x2, x3;
  float roll, pitch, yaw;

  pcl::getTranslationAndEulerAngles(po.transform, x1, x2, x3, roll, pitch, yaw);
  float cx, cy, cz, sx, sy, sz;
  cx = cos(roll);
  sx = sin(roll);
  cy = cos(pitch);
  sy = sin(pitch);
  cz = cos(pitch);
  sz = sin(pitch);

  Eigen::Vector3f a, b, c, d, e, f;

  // From Eq (6.21)
  a << 0, x1 * (-cx * sz - sx * sy * cz) + x2 * (-cx * cz + sx * sy * sz) +
              x3 * sx * cy,
      x1 * (-sx * sz + cx * sy * cz) + x2 * (-cx * sy * sz - sx * cz) +
          x3 * (-cx * cy);

  b << 0, x1 * (cx * cy * cz) + x2 * (-cx * cy * sz) + x3 * (cx * sy),
      x1 * (sx * cy * cz) + x2 * (-sx * cy * sz) + x3 * (sx * sy);

  c << 0,
      x1 * (-sx * cz - cx * sy * sz) + x2 * (sx * sz - cx * sy * cz) + x3 * 0,
      x1 * (cx * cz - sx * sy * sz) + x2 * (-sx * sy * cz - cx * sz) + x3 * 0;

  d << x1 * (-cy * cz) + x2 * (cy * sz) + x3 * (sy),
      x1 * (-sx * sy * cz) + x2 * (sx * sy * sz) + x3 * (sx * cy),
      x1 * (cx * sy * cz) + x2 * (-cx * sy * sz) + x3 * (-cx * cy);

  e << x1 * (sy * sz) + x2 * (sy * cz) + x3 * 0,
      x1 * (-sx * cy * sz) + x2 * (-sx * cy * cz) + x3 * 0,
      x1 * (cx * cy * sz) + x2 * (cx * cy * cz) + x3 * 0;

  f << x1 * (-cy * cz) + x2 * (cy * sz) + x3 * 0,
      x1 * (-cx * sz - sx * sy * cz) + x2 * (-cx * cz + sx * sy * sz) + x3 * 0,
      x1 * (-sx * sz + cx * sy * cz) + x2 * (-cx * sy * sz - sx * cz) + x3 * 0;

  // From Eq (6.20)
  Hessian3D.block<3, 1>(9, 3) = a;
  Hessian3D.block<3, 1>(12, 3) = b;
  Hessian3D.block<3, 1>(15, 3) = c;
  Hessian3D.block<3, 1>(9, 4) = b;
  Hessian3D.block<3, 1>(12, 4) = d;
  Hessian3D.block<3, 1>(15, 4) = e;
  Hessian3D.block<3, 1>(9, 5) = c;
  Hessian3D.block<3, 1>(12, 5) = e;
  Hessian3D.block<3, 1>(15, 5) = f;
}

// Initialize voxel grid with mean and sigma
void NdtLib::setMap(pcl::PointCloud<pcl::PointXYZ>::Ptr map, float resolution) {
  m_resolution = resolution;
  m_voxelGrid.setInputCloud(map);
  m_voxelGrid.setLeafSize(resolution, resolution, resolution);
  m_voxelGrid.filter(true);
}

Pose NdtLib::getCurrentPoseEstimate() { return m_poseEstimate; }

// Set params for convergence and outliers
void NdtLib::setParams(size_t maxIter, float epsilon, float outlierRatio) {
  m_maxIterations = maxIter;
  m_Epsilon = epsilon;
  m_outlierRatio = outlierRatio;
}

NdtLib::~NdtLib() {}

}  // namespace ndt_matching
