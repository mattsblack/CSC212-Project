#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rviz_visual_tools/rviz_visual_tools.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class PathPublisher : public rclcpp::Node {
public:
  PathPublisher() : Node("path_publisher") {
    // Set QoS profile to match the /odom topic
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    
    // Subscribe to odometry messages with compatible QoS
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", qos_profile, std::bind(&PathPublisher::odomCallback, this, std::placeholders::_1));

    // Create a timer to publish the path at a fixed rate (e.g., 10 Hz)
    publish_timer_ = this->create_wall_timer(100ms, std::bind(&PathPublisher::timerCallback, this));
  }

  // Call this method after the node is fully constructed (i.e., after shared_ptr is created)
  void init() {
    visual_tools_ = std::make_unique<rviz_visual_tools::RvizVisualTools>(
      "map", "/slam_toolbox/graph_visualization", rclcpp::Node::shared_from_this());
    visual_tools_->deleteAllMarkers();
    visual_tools_->trigger();
  }

private:
  // Accumulate odometry data
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received odometry data");

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = msg->header;
    pose_stamped.pose = msg->pose.pose;
    
    RCLCPP_INFO(this->get_logger(), "Pose: x = %.2f, y = %.2f", pose_stamped.pose.position.x, pose_stamped.pose.position.y);
    path_.push_back(pose_stamped);
  }

  // Publish path at a fixed rate
  void timerCallback() {
    // Convert PoseStamped to Eigen format
    EigenSTL::vector_Isometry3d eigen_path;
    for (size_t i = 1; i < path_.size(); ++i) {
      Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
      transform.translation() = Eigen::Vector3d(
          path_[i].pose.position.x, 
          path_[i].pose.position.y, 
          path_[i].pose.position.z
      );

      // Only add if the distance to the last point is significant (adjust threshold as needed)
      double dx = path_[i].pose.position.x - path_[i - 1].pose.position.x;
      double dy = path_[i].pose.position.y - path_[i - 1].pose.position.y;
      if (std::hypot(dx, dy) > 0.1) {
          eigen_path.push_back(transform);
      }
    }

    if (!eigen_path.empty()) {
      visual_tools_->publishPath(eigen_path, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::XXXLARGE);
      visual_tools_->trigger();
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::unique_ptr<rviz_visual_tools::RvizVisualTools> visual_tools_;
  std::vector<geometry_msgs::msg::PoseStamped> path_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathPublisher>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

