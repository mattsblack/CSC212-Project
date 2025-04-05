// Copyright 2021 iRobot Corporation. All Rights Reserved.

#pragma once

#include <atomic>
#include <memory>
#include <mutex>

#include "create3_straight_coverage/behaviors/behavior.hpp"
#include "create3_examples_msgs/action/coverage.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "irobot_create_msgs/action/dock.hpp"
#include "irobot_create_msgs/action/undock.hpp"
#include "irobot_create_msgs/msg/dock_status.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "irobot_create_msgs/msg/ir_opcode.hpp"
#include "irobot_create_msgs/msg/kidnap_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace create3_straight_coverage {

/**
 * @brief Main ROS2 node for the Create3 straight line coverage algorithm
 * 
 * This node provides an action server for the coverage action that implements
 * a straight-line movement pattern with obstacle avoidance behaviors for area
 * coverage. It manages robot navigation, docking, undocking, and handles
 * various sensor inputs like hazard detection and odometry.
 */
class Create3StraightCoverageNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the Create3StraightCoverageNode
     * 
     * Initializes all ROS2 interfaces including action server/clients,
     * publishers, subscribers, and parameters. Sets up connections to
     * robot sensors and actuators.
     */
    Create3StraightCoverageNode();

private:
    using CoverageAction = create3_examples_msgs::action::Coverage;
    using GoalHandleCoverage = rclcpp_action::ServerGoalHandle<CoverageAction>;

    using DockAction = irobot_create_msgs::action::Dock;
    using UndockAction = irobot_create_msgs::action::Undock;
    using DockMsg = irobot_create_msgs::msg::DockStatus;
    using HazardMsg = irobot_create_msgs::msg::HazardDetectionVector;
    using KidnapMsg = irobot_create_msgs::msg::KidnapStatus;
    using OdometryMsg = nav_msgs::msg::Odometry;
    using OpCodeMsg = irobot_create_msgs::msg::IrOpcode;
    using TwistMsg = geometry_msgs::msg::Twist;

    /**
     * @brief Set up and verify robot reflexes configuration
     * 
     * Checks if built-in robot reflexes are properly configured and activates
     * them if needed. Reflexes are robot safety features that automatically 
     * respond to hazards like bumps, cliffs, or wheel drops.
     * 
     * @return bool True if robot has reflexes enabled, false otherwise
     */
    bool reflexes_setup();

    /**
     * @brief Check if the node is ready to start coverage
     * 
     * Verifies that all required communication channels with the robot
     * are established, including publishers, subscribers, action clients,
     * and parameter services. This ensures the robot is properly connected
     * before beginning a coverage operation.
     * 
     * @return bool True if all required connections are established, false otherwise
     */
    bool ready_to_start();

    /**
     * @brief Handle coverage action goal requests
     * 
     * Callback for new coverage action goals. Validates the request and
     * checks if the robot is in a valid state to start coverage.
     * Ensures the robot isn't kidnapped and that no other coverage
     * operation is currently running.
     * 
     * @param uuid Unique identifier for the goal request
     * @param goal The requested coverage goal parameters
     * @return rclcpp_action::GoalResponse ACCEPT_AND_EXECUTE if the goal is accepted,
     *         REJECT otherwise
     */
    rclcpp_action::GoalResponse
    handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const CoverageAction::Goal> goal);

    /**
     * @brief Handle cancellation requests for coverage action
     * 
     * Callback for cancellation of an active coverage action goal.
     * Always accepts cancellation requests to allow the user to
     * stop the robot's coverage operation at any time.
     * 
     * @param goal_handle Handle to the active goal being cancelled
     * @return rclcpp_action::CancelResponse ACCEPT indicating cancellation is possible
     */
    rclcpp_action::CancelResponse
    handle_cancel(
        const std::shared_ptr<GoalHandleCoverage> goal_handle);

    /**
     * @brief Handle accepted coverage goals
     * 
     * Called when a goal has been accepted. This function starts a new
     * thread to execute the coverage action without blocking the main
     * ROS executor. The separate thread allows the node to continue
     * processing other requests and messages during coverage execution.
     * 
     * @param goal_handle Handle to the accepted goal
     */
    void handle_accepted(const std::shared_ptr<GoalHandleCoverage> goal_handle);

    /**
     * @brief Execute the coverage action
     * 
     * Main execution function for the coverage action. Creates and runs
     * the state machine, monitors for cancellation and kidnap events, and
     * publishes feedback during execution. This function runs in a separate
     * thread for non-blocking operation.
     * 
     * @param goal_handle Handle to the active goal being executed
     */
    void execute(const std::shared_ptr<GoalHandleCoverage> goal_handle);

    /**
     * @brief Callback for dock status messages
     * 
     * Updates the node's knowledge of the robot's docking state.
     * This information is critical for determining initial behaviors
     * and for tracking successful completion of docking operations.
     * 
     * @param msg Shared pointer to the received dock status message
     */
    void dock_callback(DockMsg::ConstSharedPtr msg);

    /**
     * @brief Callback for hazard detection messages
     * 
     * Updates the node's knowledge of detected hazards around the robot.
     * Hazards include obstacles, cliffs, and bump events that require
     * the robot to modify its behavior for safety.
     * 
     * @param msg Shared pointer to the received hazard detection message
     */
    void hazards_callback(HazardMsg::ConstSharedPtr msg);

    /**
     * @brief Callback for IR opcode messages
     * 
     * Updates the node's knowledge of IR sensor readings, particularly
     * for dock detection. IR opcodes provide information about virtual
     * walls, dock beacons, and other IR-emitting devices in the environment.
     * 
     * @param msg Shared pointer to the received IR opcode message
     */
    void ir_opcode_callback(OpCodeMsg::ConstSharedPtr msg);

    /**
     * @brief Callback for kidnap status messages
     * 
     * Updates the node's knowledge of whether the robot has been moved
     * unexpectedly (kidnapped). Kidnap detection is important for aborting
     * coverage operations when the robot's position has been manually changed.
     * 
     * @param msg Shared pointer to the received kidnap status message
     */
    void kidnap_callback(KidnapMsg::ConstSharedPtr msg);

    /**
     * @brief Callback for odometry messages
     * 
     * Updates the node's knowledge of the robot's position and orientation.
     * Odometry data is essential for tracking the robot's movement, measuring
     * traveled distance, and determining the robot's pose for navigation.
     * 
     * @param msg Shared pointer to the received odometry message
     */
    void odom_callback(OdometryMsg::ConstSharedPtr msg);

    /// ID of the last reported behavior for feedback
    int32_t m_last_behavior;

    /// Control loop rate in Hz
    double m_rate_hz;
    
    /// Time window in milliseconds for collecting IR opcodes
    int m_opcodes_buffer_ms;

    /// Flag indicating if a coverage action is currently active
    std::atomic<bool> m_is_running;
    
    /// Mutex for thread-safe access to sensor data
    std::mutex m_mutex;

    /// Flag indicating if dock messages have been received
    std::atomic<bool> m_dock_msgs_received;
    
    /// Latest dock status message
    DockMsg m_last_dock;
    
    /// Latest hazard detections
    HazardMsg m_last_hazards;
    
    /// Latest kidnap status
    KidnapMsg m_last_kidnap;
    
    /// Latest odometry data
    OdometryMsg m_last_odom;
    
    /// Recent IR opcodes within buffer window
    std::vector<OpCodeMsg> m_last_opcodes;
    
    /// Time when opcodes buffer was last cleared
    rclcpp::Time m_last_opcodes_cleared_time;

    /// Server for coverage action
    rclcpp_action::Server<CoverageAction>::SharedPtr m_coverage_action_server;

    /// Client for docking action
    rclcpp_action::Client<DockAction>::SharedPtr m_dock_action_client;
    
    /// Client for undocking action
    rclcpp_action::Client<UndockAction>::SharedPtr m_undock_action_client;

    /// Publisher for velocity commands
    rclcpp::Publisher<TwistMsg>::SharedPtr m_cmd_vel_publisher;

    /// Client for reflexes parameter service
    rclcpp::AsyncParametersClient::SharedPtr m_reflexes_param_client;

    /// Subscription for dock status
    rclcpp::Subscription<DockMsg>::SharedPtr m_dock_subscription;
    
    /// Subscription for hazard detection
    rclcpp::Subscription<HazardMsg>::SharedPtr m_hazards_subscription;
    
    /// Subscription for kidnap status
    rclcpp::Subscription<KidnapMsg>::SharedPtr m_kidnap_subscription;
    
    /// Subscription for odometry
    rclcpp::Subscription<OdometryMsg>::SharedPtr m_odom_subscription;
    
    /// Subscription for IR opcodes
    rclcpp::Subscription<OpCodeMsg>::SharedPtr m_ir_opcode_subscription;
};

} // namespace create3_straight_coverage
