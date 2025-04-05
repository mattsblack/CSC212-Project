// Copyright 2021 iRobot Corporation. All Rights Reserved.

#pragma once

#include "create3_spiral_coverage/behaviors/behavior.hpp"
#include "create3_examples_msgs/action/coverage.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace create3_spiral_coverage {

/**
 * @brief Behavior that responds to hazards by backing up the robot
 * 
 * When hazards like obstacles are detected, this behavior commands the robot to
 * back up a short distance to clear the hazard. It's typically used as a reactive
 * safety mechanism within other behaviors.
 */
class ReflexBehavior : public Behavior
{
public:
    using TwistMsg = geometry_msgs::msg::Twist;  ///< Type for velocity commands

    /**
     * @brief Configuration parameters for the reflex behavior
     */
    struct Config
    {
        double backup_distance {0.05};  ///< Distance to back up in meters
        double linear_vel {0.1};        ///< Backup speed in meters per second
        rclcpp::Duration clear_hazard_time {rclcpp::Duration(std::chrono::seconds(2))};  ///< Maximum time for hazard clearing
    };

    /**
     * @brief Constructor for ReflexBehavior
     * 
     * @param config Configuration parameters for reflexive backing up
     * @param cmd_vel_publisher Publisher for velocity commands
     * @param logger ROS logger for output messages
     * @param clock ROS clock for time tracking
     */
    ReflexBehavior(
        Config config,
        rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_publisher,
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock);

    /**
     * @brief Default destructor
     */
    ~ReflexBehavior() = default;

    /**
     * @brief Execute one cycle of the reflex behavior
     * 
     * Sends reverse velocity commands to back up the robot until the
     * hazard is cleared, backup distance is reached, or a backup limit is hit.
     * 
     * @param data Current robot sensor and state data
     * @return State RUNNING if still backing up, SUCCESS if hazard cleared,
     *               FAILURE if hazard couldn't be cleared
     */
    State execute(const Data & data) override;

    /**
     * @brief Get the behavior's unique identifier
     * 
     * @return int32_t DRIVE_STRAIGHT identifier (uses same ID as drive straight for compatibility)
     */
    int32_t get_id() const override { return create3_examples_msgs::action::Coverage::Feedback::DRIVE_STRAIGHT; }

private:
    /**
     * @brief Check if a backup limit hazard has been detected
     * 
     * Determines if the robot has reached a point where it cannot back up further,
     * such as when the rear bump sensor is triggered.
     * 
     * @param hazards Collection of currently detected hazards
     * @return bool True if a backup limit has been reached, false otherwise
     */
    bool backup_limit_reached(const irobot_create_msgs::msg::HazardDetectionVector& hazards);

    Config m_config;  ///< Behavior configuration parameters

    bool m_first_run;  ///< Flag for first execution to capture initial position
    rclcpp::Time m_start_time;  ///< Time when behavior started executing
    geometry_msgs::msg::Point m_initial_position;  ///< Starting position for distance calculation

    rclcpp::Publisher<TwistMsg>::SharedPtr m_cmd_vel_publisher;  ///< Publisher for velocity commands
    rclcpp::Logger m_logger;  ///< Logger for messages
    rclcpp::Clock::SharedPtr m_clock;  ///< Clock for time tracking
};

} // namespace create3_spiral_coverage
