#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rviz_visual_tools/rviz_visual_tools.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

/**
 * @class PathPublisher
 * @brief Visualizes the robot's movement path in RViz
 *
 * This class subscribes to odometry data, collects the robot's movement history,
 * and publishes a visual path representation for RViz. It helps in visualizing 
 * the coverage pattern and trajectory of the robot during operation.
 */
class PathPublisher : public rclcpp::Node {
public:
  /**
   * @brief Constructor for PathPublisher
   * 
   * Initializes the node with necessary subscriptions and timers. Sets up the
   * odometry subscription with appropriate QoS settings to match the robot's
   * odometry publisher. Creates a timer for periodic path visualization updates.
   */
  PathPublisher() : Node("path_publisher") {
    // Set QoS profile to match the /odom topic
    // Using sensor data profile which is typically used for odometry to handle potentially
    // high frequency, best-effort delivery of position updates
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    
    // Subscribe to odometry messages with compatible QoS
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", qos_profile, std::bind(&PathPublisher::odomCallback, this, std::placeholders::_1));

    // Create a timer to publish the path at a fixed rate (e.g., 10 Hz)
    // This prevents overloading RViz with too frequent updates while ensuring
    // the path visualization remains current
    publish_timer_ = this->create_wall_timer(100ms, std::bind(&PathPublisher::timerCallback, this));
  }

  /**
   * @brief Initialize visualization tools
   * 
   * This method must be called after the node is fully constructed. It initializes
   * the RViz visualization tools with proper frame references and clears any
   * existing markers to ensure a clean visualization.
   * 
   * @note Call this method only after creating a shared_ptr to the node
   */
  void init() {
    // Create visualization tools with map frame reference
    // The shared_from_this() call requires that this node already be managed by a shared_ptr
    visual_tools_ = std::make_unique<rviz_visual_tools::RvizVisualTools>(
      "map",                              // Fixed frame for visualization
      "/slam_toolbox/graph_visualization", // Topic for the markers
      rclcpp::Node::shared_from_this());
    
    // Clear any existing visualizations to start fresh
    visual_tools_->deleteAllMarkers();
    visual_tools_->trigger();
  }

private:
  /**
   * @brief Process incoming odometry messages
   * 
   * This callback runs each time a new odometry message is received. It extracts
   * the pose information and adds it to the path history for later visualization.
   * 
   * @param msg Shared pointer to the received odometry message
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received odometry data");

    // Convert odometry message to PoseStamped for path tracking
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = msg->header;  // Preserve timestamp and frame information
    pose_stamped.pose = msg->pose.pose; // Extract the pose component
    
    // Log the position for debugging purposes
    RCLCPP_INFO(this->get_logger(), "Pose: x = %.2f, y = %.2f", 
                pose_stamped.pose.position.x, pose_stamped.pose.position.y);
    
    // Add the pose to our path history
    path_.push_back(pose_stamped);
  }

  /**
   * @brief Publish the accumulated path for visualization
   * 
   * This method runs periodically to convert the accumulated pose data into
   * a format suitable for RViz visualization. It filters out poses that are
   * too close together to avoid cluttering the visualization with redundant points.
   */
  void timerCallback() {
    // Convert PoseStamped messages to Eigen format required by RViz Visual Tools
    EigenSTL::vector_Isometry3d eigen_path;
    
    // Process each point in the path (starting from index 1 to compare with previous)
    for (size_t i = 1; i < path_.size(); ++i) {
      // Create a transformation matrix representing the pose
      Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
      
      // Set the translation component from the pose position
      transform.translation() = Eigen::Vector3d(
          path_[i].pose.position.x, 
          path_[i].pose.position.y, 
          path_[i].pose.position.z
      );

      // Calculate distance from previous point to implement filtering
      double dx = path_[i].pose.position.x - path_[i - 1].pose.position.x;
      double dy = path_[i].pose.position.y - path_[i - 1].pose.position.y;
      
      // Only add points that are significantly different from the previous one
      // This reduces visual clutter and improves rendering performance
      if (std::hypot(dx, dy) > 0.1) {  // 0.1m (10cm) threshold - adjust as needed
          eigen_path.push_back(transform);
      }
    }

    // Only publish if we have path points to visualize
    if (!eigen_path.empty()) {
      // Publish the path with bright green color and large size for visibility
      visual_tools_->publishPath(eigen_path, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::XXXLARGE);
      
      // Trigger actual sending of visualization markers
      visual_tools_->trigger();
    }
  }

  // Node member variables
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;  ///< Subscription to odometry data
  rclcpp::TimerBase::SharedPtr publish_timer_;  ///< Timer for periodic visualization updates
  std::unique_ptr<rviz_visual_tools::RvizVisualTools> visual_tools_;  ///< RViz visualization toolkit
  std::vector<geometry_msgs::msg::PoseStamped> path_;  ///< Container for the robot's pose history
};

/**
 * @brief Main function for the path_publisher node
 * 
 * Initializes ROS 2, creates the PathPublisher node, and starts the executor
 * to process callbacks until the node is shut down.
 * 
 * @param argc Number of command-line arguments
 * @param argv Command-line argument values
 * @return int Exit status (0 if normal exit)
 */
int main(int argc, char **argv) {
  // Initialize ROS 2 communication
  rclcpp::init(argc, argv);
  
  // Create a shared pointer to a new PathPublisher node
  auto node = std::make_shared<PathPublisher>();
  
  // Initialize visualization tools (must be called after creating shared_ptr)
  node->init();
  
  // Process callbacks until the node is shut down
  rclcpp::spin(node);
  
  // Clean up ROS 2 resources
  rclcpp::shutdown();
  return 0;
}

