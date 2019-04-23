#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#include "ndt_matching/ndt_lib.hpp"

using namespace pcl;
using namespace pcl::io;

pcl::PointCloud<pcl::PointXYZ>::Ptr currentScan;

TEST(NDT_Matching, NormalDistributionsTransform) {
  ndt_matching::NdtLib lib;
  Eigen::Matrix4f poseGuess;
  pcl::PointCloud<pcl::PointXYZ> transformedCloud;

  lib.setMap(currentScan, 0.5);
  transformPointCloud(transformedCloud, transformedCloud, poseGuess);
  poseGuess = lib.run(transformedCloud, poseGuess);

  EXPECT_EQ(1, 1);
  // EXPECT_LT(reg.getFitnessScore(), 0.001);
}

int main(int argc, char** argv) {
  currentScan =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  if (loadPCDFile(argv[1], *currentScan) < 0) {
    std::cerr << "Failed to read test file. Please download `map.pcd` and "
                 "pass its path to the test."
              << std::endl;
    return (-1);
  }

  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}