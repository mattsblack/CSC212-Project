// Copyright 2021 iRobot Corporation. All Rights Reserved.

#pragma once

#include "create3_spiral_coverage/behaviors/behavior.hpp"
#include "create3_spiral_coverage/behaviors/reflex-behavior.hpp"
#include "create3_examples_msgs/action/coverage.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace create3_spiral_coverage {

/**
 * @brief Behavior that rotates the robot by a specified angle
 * 
 * This behavior rotates the robot in place by a target angle. It handles hazards
 * that may occur during rotation by either waiting for built-in reflexes or
 * by triggering its own reflex behavior to clear the hazard.
 */
class RotateBehavior : public Behavior
{
public:
    using TwistMsg = geometry_msgs::msg::Twist;  ///< Type for velocity commands

    /**
     * @brief Configuration parameters for the rotation behavior
     */
    struct Config
    {
        double target_rotation {0.785398};  ///< Target rotation angle in radians (default: π/4)
        double angular_vel {0.8};  ///< Angular velocity in radians per second
        bool robot_has_reflexes {false};  ///< Whether robot has built-in hazard reflexes
        size_t max_hazard_retries {10};  ///< Maximum attempts to clear hazards
        rclcpp::Duration clear_hazard_time {rclcpp::Duration(std::chrono::seconds(2))};  ///< Maximum time for hazard clearing
    };

    /**
     * @brief Constructor for RotateBehavior
     * 
     * @param config Configuration parameters for rotation
     * @param cmd_vel_publisher Publisher for velocity commands
     * @param logger ROS logger for output messages
     * @param clock ROS clock for time tracking
     */
    RotateBehavior(
        Config config,
        rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_publisher,
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock);

    /**
     * @brief Default destructor
     */
    ~RotateBehavior() = default;

    /**
     * @brief Execute one cycle of the rotation behavior
     * 
     * Handles hazard detection and sends angular velocity commands to rotate
     * the robot until the target angle is reached.
     * 
     * @param data Current robot sensor and state data
     * @return State RUNNING if still rotating, SUCCESS if target rotation achieved,
     *               FAILURE if blocked by persistent hazards
     */
    State execute(const Data & data) override;

    /**
     * @brief Get the behavior's unique identifier
     * 
     * @return int32_t ROTATE identifier from Coverage action feedback constants
     */
    int32_t get_id() const override { return create3_examples_msgs::action::Coverage::Feedback::ROTATE; }

    /**
     * @brief Get the actual rotation amount achieved
     * 
     * @return double The rotation completed in radians
     */
    double get_rotation_amount() { return m_rotation_amount; }

private:
    /**
     * @brief Handle hazards encountered during rotation
     * 
     * This method detects and responds to hazards either by waiting for
     * built-in robot reflexes or by initiating a reflex behavior to back away
     * from obstacles.
     * 
     * @param data Current robot sensor and state data
     * @return State SUCCESS if hazards cleared, RUNNING if still handling hazards,
     *               FAILURE if hazards couldn't be cleared
     */
    State handle_hazards(const Data & data);

    Config m_config;  ///< Behavior configuration parameters
    double m_rotation_amount;  ///< Actual rotation completed so far

    bool m_first_run;  ///< Flag for first execution to capture initial orientation
    rclcpp::Time m_start_time;  ///< Time when behavior started executing
    tf2::Quaternion m_initial_orientation;  ///< Starting orientation for rotation calculation

    std::unique_ptr<ReflexBehavior> m_reflex_behavior;  ///< Reflex behavior for hazard handling
    size_t m_hazards_count;  ///< Counter for hazard handling attempts

    rclcpp::Publisher<TwistMsg>::SharedPtr m_cmd_vel_publisher;  ///< Publisher for velocity commands
    rclcpp::Logger m_logger;  ///< Logger for messages
    rclcpp::Clock::SharedPtr m_clock;  ///< Clock for time tracking
};

} // namespace create3_spiral_coverage
