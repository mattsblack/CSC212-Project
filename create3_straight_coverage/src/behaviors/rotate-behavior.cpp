// Copyright 2021 iRobot Corporation. All Rights Reserved.

#include <cmath>

#include "create3_straight_coverage/behaviors/rotate-behavior.hpp"
#include "tf2/utils.h"

#include "utils.hpp"

namespace create3_straight_coverage {

/**
 * @brief Constructor for the RotateBehavior class
 *
 * Initializes a new RotateBehavior instance that rotates the robot to
 * a target orientation or by a specified angle. This behavior handles
 * hazards that may occur during rotation through its own hazard handling.
 *
 * @param config Configuration parameters including target rotation angle and angular velocity
 * @param cmd_vel_publisher Publisher for sending velocity commands to the robot
 * @param logger Logger instance for outputting status and debug information
 * @param clock Clock for tracking execution duration
 */
RotateBehavior::RotateBehavior(
    Config config,
    rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_publisher,
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock)
: m_cmd_vel_publisher(cmd_vel_publisher), m_logger(logger)
{
    m_clock = clock;
    m_config = config;

    // Initialize rotation state variables
    m_first_run = false;  // Will be set to true after initial hazard check
    m_start_time = m_clock->now();
    m_hazards_count = 0;  // Count how many times we've tried to clear hazards
}

/**
 * @brief Execute one iteration of the rotation behavior
 *
 * This method handles rotating the robot to a target orientation. It first
 * attempts to clear any hazards through reflexes, then tracks rotation progress
 * until the desired angle is reached. On first execution after hazards are cleared,
 * it records the initial orientation to measure rotation progress.
 *
 * @param data Current sensor and state data from the robot
 * @return State Indicates the current execution state:
 *         - RUNNING: Rotation is still in progress
 *         - SUCCESS: Target rotation has been achieved
 *         - FAILURE: Unable to complete rotation due to persistent hazards
 */
State RotateBehavior::execute(const Data & data)
{
    // Safety check - if we've tried to handle hazards too many times, give up
    if (m_hazards_count > m_config.max_hazard_retries) {
        RCLCPP_INFO(m_logger, "Failed to clear hazard! Too many trials");
        return State::FAILURE;
    }

    // First, handle any hazards using reflexes before attempting rotation
    State reflex_state = handle_hazards(data);
    
    // If hazards are still being handled or failed to clear, propagate that state
    if (reflex_state != State::SUCCESS) {
        return reflex_state;
    }
    
    // Hazards cleared - cleanup reflex behavior
    m_reflex_behavior.reset();

    // After hazards are cleared, initialize rotation tracking if first execution
    if (!m_first_run) {
        m_first_run = true;
        
        // Record initial orientation as a quaternion to measure rotation against
        tf2::convert(data.pose.orientation, m_initial_orientation);

        RCLCPP_DEBUG(m_logger, "Rotation initial yaw: %f", tf2::getYaw(m_initial_orientation));
    }

    // Convert current orientation quaternion to tf2 format for processing
    tf2::Quaternion current_orientation;
    tf2::convert(data.pose.orientation, current_orientation);
    
    // Calculate relative rotation from starting orientation
    // This quaternion represents how much we've rotated from the initial orientation
    tf2::Quaternion relative_orientation = current_orientation * m_initial_orientation.inverse();

    // Extract the yaw (rotation around Z axis) from the quaternion
    double relative_yaw = tf2::getYaw(relative_orientation);
    
    // Check if we've reached or exceeded our target rotation angle
    if (std::abs(relative_yaw) >= std::abs(m_config.target_rotation)) {
        RCLCPP_INFO(m_logger, "Rotation completed: from %f to %f",
            tf2::getYaw(m_initial_orientation), tf2::getYaw(current_orientation));
        return State::SUCCESS;
    }

    // Log rotation progress for debugging
    RCLCPP_DEBUG(m_logger, "Rotating: current orientation %f progress %f/%f",
        tf2::getYaw(current_orientation), relative_yaw, m_config.target_rotation);

    // Continue rotating by publishing angular velocity command
    auto twist_msg = std::make_unique<TwistMsg>();
    
    // Set angular velocity with the proper sign to rotate in the correct direction
    // copysign returns the magnitude of angular_vel with the sign of target_rotation
    twist_msg->angular.z = std::copysign(m_config.angular_vel, m_config.target_rotation);
    m_cmd_vel_publisher->publish(std::move(twist_msg));

    return State::RUNNING;
}

/**
 * @brief Handle hazards during rotation by using reflexes
 *
 * This method checks for active hazards and either waits for built-in robot
 * reflexes to resolve them or initiates a reflex behavior to clear the hazard.
 * It helps ensure the robot can rotate safely even when obstacles are encountered.
 *
 * @param data Current sensor and state data from the robot
 * @return State Indicates the hazard handling state:
 *         - RUNNING: Still handling hazards
 *         - SUCCESS: Hazards cleared successfully
 *         - FAILURE: Unable to clear hazards within time limit
 */
State RotateBehavior::handle_hazards(const Data & data)
{
    // If no hazards and no active reflex behavior, everything is fine
    if (!is_front_hazard_active(data.hazards) && !m_reflex_behavior) {
        return State::SUCCESS;
    }

    // If the robot has built-in reflexes, let them handle hazards
    if (m_config.robot_has_reflexes) {
        // Wait for built-in reflexes to clear the hazards, but not forever
        // Abort if they take too long (timeout protection)
        if (m_clock->now() - m_start_time > m_config.clear_hazard_time) {
            RCLCPP_INFO(m_logger, "Aborting ROTATE because initial hazard is not getting cleared");
            return State::FAILURE;
        }

        // Return SUCCESS if hazards cleared, or RUNNING if still active
        if (!is_front_hazard_active(data.hazards)) {
            return State::SUCCESS;
        } else {
            return State::RUNNING;
        }
    } else {
        // If robot doesn't have reflexes, we need to run our own reflex behavior

        // Initialize the reflex behavior if it doesn't exist yet
        if (!m_reflex_behavior) {
            m_hazards_count++;
            RCLCPP_INFO(m_logger, "Starting reflex behavior to clear hazard");

            // Configure and create a reflex behavior instance
            auto config = ReflexBehavior::Config();
            config.clear_hazard_time = m_config.clear_hazard_time;
            config.backup_distance = 0.05;  // Small backup distance for rotation
            config.linear_vel = 0.1;        // Slow velocity for controlled backup
            m_reflex_behavior = std::make_unique<ReflexBehavior>(config, m_cmd_vel_publisher, m_logger, m_clock);
        }

        // Execute one cycle of the reflex behavior and return its state
        auto reflex_state = m_reflex_behavior->execute(data);
        return reflex_state;
    }
}

} // namespace create3_straight_coverage
