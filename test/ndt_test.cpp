#include <chrono>

#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#include "ndt_matching/ndt_lib.hpp"

using namespace pcl;
using namespace pcl::io;

class NdtUnitTest : public ::testing::Test {
 protected:

  pcl::PointCloud<pcl::PointXYZ>::Ptr currentScan;
  ndt_matching::NdtLib lib;
  ndt_matching::Pose poseGuess;
  ndt_matching::Pose estimatedPose;
  std::vector<float> poseGuessVec;


  NdtUnitTest(){

    poseGuessVec = std::vector<float>(6,0.0);
    currentScan =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    const char* pcdFile = std::getenv("PCD");
    if (loadPCDFile(pcdFile, *currentScan) < 0) {
      std::cerr << "Failed to read test file. Please download `map.pcd` and "
                   "pass its path to the test."
                << std::endl;
      exit(-1);
    }
  }

  ~NdtUnitTest() override {
  }

  void SetUp() override {
    // Downsampling
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(currentScan);
    sor.setLeafSize(5.0f, 5.0f, 5.0f);
    sor.filter(*currentScan);

    poseGuessVec[0] = 1.0;
    poseGuessVec[1] = 1.0;
    poseGuessVec[2] = 0.1;
    poseGuessVec[3] = 0.2;
    poseGuessVec[4] = 0.1;
    poseGuessVec[5] = 0.2;
    pcl::getTransformation(poseGuessVec[0], poseGuessVec[1], poseGuessVec[2],
                           poseGuessVec[3], poseGuessVec[4], poseGuessVec[5],
                           poseGuess.transform);
    lib.setMap(currentScan, 0.5);
    lib.setParams(50, 0.01, 0.2);
  }

  void TearDown() override {
  }
};

TEST_F(NdtUnitTest, CorrectnessTest) {
  pcl::PointCloud<pcl::PointXYZ> transformedCloud;
  transformedCloud = *currentScan;
  transformPointCloud(transformedCloud, transformedCloud,
                      poseGuess.transform.inverse());

  estimatedPose = lib.run(transformedCloud, poseGuess);
  std::vector<float> estimatedPoseVec(6, 0.0);

  pcl::getTranslationAndEulerAngles(estimatedPose.transform,
                                    estimatedPoseVec[0], estimatedPoseVec[1],
                                    estimatedPoseVec[2], estimatedPoseVec[3],
                                    estimatedPoseVec[4], estimatedPoseVec[5]);

  EXPECT_NEAR(estimatedPoseVec[0], poseGuessVec[0], 0.01);
  EXPECT_NEAR(estimatedPoseVec[1], poseGuessVec[1], 0.01);
  EXPECT_NEAR(estimatedPoseVec[2], poseGuessVec[2], 0.01);
  EXPECT_NEAR(estimatedPoseVec[3], poseGuessVec[3], 0.01);
  EXPECT_NEAR(estimatedPoseVec[4], poseGuessVec[4], 0.01);
  EXPECT_NEAR(estimatedPoseVec[5], poseGuessVec[5], 0.01);
}

TEST_F(NdtUnitTest, PerformanceTest) {

  lib.setParams(1, 0.0001, 0.2);
  pcl::PointCloud<pcl::PointXYZ> transformedCloud;
  transformedCloud = *currentScan;
  transformPointCloud(transformedCloud, transformedCloud,
                      poseGuess.transform.inverse());
  auto start = std::chrono::steady_clock::now();
  estimatedPose = lib.run(transformedCloud, poseGuess);
  auto end = std::chrono::steady_clock::now();
  auto total_runtime =
      std::chrono::duration<double, std::milli>(end - start).count();
  std::cout << "Time taken by ndt_matching : " << total_runtime << " ms"
            << std::endl;

  EXPECT_LT(total_runtime,10000);

}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}