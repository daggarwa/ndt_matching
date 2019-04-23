#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include "std_msgs/msg/string.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "ndt_matching/ndt_lib.hpp"

void print_usage() {
  printf("Usage for listener app:\n");
  printf("listener [-t topic_name] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf(
      "-t topic_name : Specify the topic on which to subscribe. Defaults to "
      "chatter.\n");
}

// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Listener : public rclcpp::Node {
 public:
  explicit Listener(const std::string& topic_name = "filtered_points",
                    const std::string& topic_name2 = "cloud_pcd",
                    const std::string& topic_name3 = "current_pose")
      : Node("listener") {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for
    // zero-copy transport.
    auto callback =
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void {
      RCLCPP_INFO(this->get_logger(), "I heard bag : [%s]",
                  msg->header.frame_id.c_str());
      pcl::fromROSMsg(*msg, bag_pclouds);
      ndt_matching::Pose tf = ndt.run(bag_pclouds, currentPose);
      publishPoseMsg(tf);
      // TODO:
      // here you call NdtLib function and pass in the msg as input
      // return a pose message and publish it as
      // https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/PoseStamped.msg
    };

    auto callback2 =
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void {
      pcl::PointCloud<pcl::PointXYZ> map;
      RCLCPP_INFO(this->get_logger(), "I heard map: [%s]",
                  msg->header.frame_id.c_str());
      pcl::fromROSMsg(*msg, map);
      mapPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr(
          new pcl::PointCloud<pcl::PointXYZ>(map));
      ndt.setMap(mapPtr, 0.5);
      ndt.setParams(50, 0.01, 0.2);

      // TODO: here you get your map point cloud (one time only)
    };

    auto callback3 =
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void {
      RCLCPP_INFO(this->get_logger(), "current pose : [%s]",
                  msg->header.frame_id.c_str());
      setCurrentPose(msg);
    };

    // Create a subscription to the topic which can be matched with one or more
    // compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be
    // compatible:
    // they must have compatible Quality of Service policies.
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(topic_name,
                                                              callback);
    sub2_ = create_subscription<sensor_msgs::msg::PointCloud2>(topic_name2,
                                                               callback2);
    sub3_ = create_subscription<geometry_msgs::msg::PoseStamped>(topic_name3,
                                                                 callback3);
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("estimated_pose");
    // TODO: create a pose publisher, see for reference
    // https://github.com/ros2/demos/blob/master/demo_nodes_cpp/src/topics/talker.cpp
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

    pcl::getTranslationAndEulerAngles(pose.transform, x, y, z, roll, pitch, yaw);
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

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  // Initialize any global resources needed by the middleware and the client
  // library.
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  // Parse the command line options.
  auto topic = std::string("points_raw");
  char* cli_option = rcutils_cli_get_option(argv, argv + argc, "-t");
  if (nullptr != cli_option) {
    topic = std::string(cli_option);
  }

  // Create a node.
  auto node = std::make_shared<Listener>(topic);

  // spin will block until work comes in, execute work as it becomes
  // available,
  // and keep blocking.
  // It will only be interrupted by Ctrl-C.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
