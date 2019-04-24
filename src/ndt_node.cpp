#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "ndt_matching/ndt_lib.hpp"

// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Listener : public rclcpp::Node {
 public:
  explicit Listener(const std::string& topic_name = "filtered_points",
                    const std::string& topic_name2 = "map",
                    const std::string& topic_name3 = "current_pose")
      : Node("listener") {
    // Callback for processing current scan coming on filtered_points topic and
    // calling ndt_lib function to run NormalDistributionsTransform matching of current scan
    // with map
    auto callback =
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void {
      RCLCPP_INFO(this->get_logger(),
                  "Inside callback for current scan from bag file");
      pcl::fromROSMsg(*msg, bag_pclouds);
      ndt_matching::Pose tf = ndt.run(bag_pclouds, currentPose);
      publishPoseMsg(tf);
    };

    // Callback for processing map point cloud coming on map topic
    auto callback2 =
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void {
      pcl::PointCloud<pcl::PointXYZ> map;
      RCLCPP_INFO(this->get_logger(), "Inside callback for published map pcd");
      pcl::fromROSMsg(*msg, map);
      mapPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr(
          new pcl::PointCloud<pcl::PointXYZ>(map));
      ndt.setMap(mapPtr, 0.5);
    };

    // Callback for setting current pose of robot coming on current_pose topic
    auto callback3 =
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void {
      RCLCPP_INFO(this->get_logger(),
                  "Inside callback for published current pose ");
      setCurrentPose(msg);
    };

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(topic_name,
                                                              callback);
    sub2_ = create_subscription<sensor_msgs::msg::PointCloud2>(topic_name2,
                                                               callback2);
    sub3_ = create_subscription<geometry_msgs::msg::PoseStamped>(topic_name3,
                                                                 callback3);
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("estimated_pose");
    ndt.setParams(50, 0.01, 0.2);
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub2_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub3_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr mapPtr;
  pcl::PointCloud<pcl::PointXYZ> bag_pclouds;

  geometry_msgs::msg::PoseStamped ndt_pose_msg;

  ndt_matching::NdtLib ndt;
  ndt_matching::Pose currentPose;

  void publishPoseMsg(ndt_matching::Pose& pose) {
    tf2::Quaternion ndt_q;
    // Update ndt_pose
    float x;
    float y;
    float z;
    float roll = 0.0;
    float pitch = 0.0;
    float yaw = 0.0;

    pcl::getTranslationAndEulerAngles(pose.transform, x, y, z, roll, pitch,
                                      yaw);
    ndt_q.setRPY(roll, pitch, yaw);

    ndt_pose_msg.header.frame_id = "/map";
    ndt_pose_msg.header.stamp = this->now();
    ndt_pose_msg.pose.position.x = x;
    ndt_pose_msg.pose.position.y = y;
    ndt_pose_msg.pose.position.z = z;
    ndt_pose_msg.pose.orientation.x = ndt_q.x();
    ndt_pose_msg.pose.orientation.y = ndt_q.y();
    ndt_pose_msg.pose.orientation.z = ndt_q.z();
    ndt_pose_msg.pose.orientation.w = ndt_q.w();

    pub_->publish(ndt_pose_msg);
  }

  void setCurrentPose(
      const geometry_msgs::msg::PoseStamped::SharedPtr poseMsg) {
    tf2::Quaternion q(poseMsg->pose.orientation.x, poseMsg->pose.orientation.y,
                      poseMsg->pose.orientation.z, poseMsg->pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    pcl::getTransformation(poseMsg->pose.position.x, poseMsg->pose.position.y,
                           poseMsg->pose.position.z, roll, pitch, yaw,
                           currentPose.transform);
  }
};

int main(int argc, char* argv[]) {
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  // Create a node.
  auto node = std::make_shared<Listener>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
