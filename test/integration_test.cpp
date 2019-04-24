#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "std_msgs/msg/string.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class IntegrationTest : public rclcpp::Node {
 public:
  explicit IntegrationTest(const std::string& topic_name = "map",
                           const std::string& topic_name2 = "filtered_points",
                           const std::string& topic_name3 = "current_pose")
      : Node("IntegrationTest") {
    auto callback =
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void {
      RCLCPP_INFO(this->get_logger(), "Estimated pose callback");
      ndtPoseReceived = true;
    };

    auto timer_callback = [this]() -> void {
      RCLCPP_INFO(this->get_logger(), "Inside timer_callback");
      this->publishMap();
      this->publishCurrentScan();
      this->publishCurrentPoseMsg();
    };
    timer_ = this->create_wall_timer(5000ms, timer_callback);

    // map
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic_name);
    // filtered pts
    pub2_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic_name2);
    // current_pose
    pub3_ = create_publisher<geometry_msgs::msg::PoseStamped>(topic_name3);
    // estimated_pose
    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "estimated_pose", callback);
  }

  void publishCurrentPoseMsg() {
    geometry_msgs::msg::PoseStamped current_pose_msg;
    tf2::Quaternion current_q;

    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    float roll = 0.0;
    float pitch = 0.0;
    float yaw = 0.0;

    current_q.setRPY(roll, pitch, yaw);

    current_pose_msg.header.frame_id = "/map";
    current_pose_msg.header.stamp = this->now();
    current_pose_msg.pose.position.x = x;
    current_pose_msg.pose.position.y = y;
    current_pose_msg.pose.position.z = z;
    current_pose_msg.pose.orientation.x = current_q.x();
    current_pose_msg.pose.orientation.y = current_q.y();
    current_pose_msg.pose.orientation.z = current_q.z();
    current_pose_msg.pose.orientation.w = current_q.w();

    pub3_->publish(current_pose_msg);
  }

  void publishMap() {
    // Create a PointCloud2
    sensor_msgs::msg::PointCloud2 map;
    // Fill some internals of the PoinCloud2 like the header/width/height ..
    map.height = 1;
    map.width = 4;
    sensor_msgs::PointCloud2Modifier modifier(map);
    modifier.setPointCloud2Fields(4, "x", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "rgb",
                                  1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    // You can then reserve / resize as usual
    modifier.resize(100);
    pub_->publish(map);
  }

  void publishCurrentScan() {
    // Create a PointCloud2
    sensor_msgs::msg::PointCloud2 currentScan;
    // Fill some internals of the PoinCloud2 like the header/width/height ...
    currentScan.height = 1;
    currentScan.width = 4;

    sensor_msgs::PointCloud2Modifier modifier(currentScan);
    modifier.setPointCloud2Fields(4, "x", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "rgb",
                                  1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    // You can then reserve / resize as usual
    modifier.resize(10);
    pub2_->publish(currentScan);
  }

  bool getNdtPoseReceived() const { return ndtPoseReceived; }

 private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub2_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub3_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool ndtPoseReceived = false;
};

TEST(IntegrationTest, communicationTest) {
  auto node = std::make_shared<IntegrationTest>();
  while (!node->getNdtPoseReceived()) {
    rclcpp::spin_some(node);
  }
  EXPECT_EQ(node->getNdtPoseReceived(), true);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
  rclcpp::shutdown();
  return 0;
}
