// Copyright 2021 iRobot Corporation. All Rights Reserved.

#pragma once

#include "create3_straight_coverage/behaviors/behavior.hpp"
#include "create3_examples_msgs/action/coverage.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace create3_straight_coverage {

/**
 * @brief Behavior that drives the robot in a straight line
 * 
 * This behavior commands the robot to move forward in a straight line until
 * a termination condition is met: maximum distance is reached, an obstacle is detected,
 * or the dock is detected in front of the robot.
 */
class DriveStraightBehavior : public Behavior
{
public:
    using OpCodeMsg = irobot_create_msgs::msg::IrOpcode;  ///< Type for IR sensor opcodes
    using TwistMsg = geometry_msgs::msg::Twist;  ///< Type for velocity commands

    /**
     * @brief Configuration parameters for the drive straight behavior
     */
    struct Config
    {
        double max_distance {5};    ///< Maximum distance to travel in meters
        double min_distance {0.25}; ///< Minimum distance to travel in meters
        double linear_vel {0.3};    ///< Forward velocity in meters per second
    };

    /**
     * @brief Constructor for DriveStraightBehavior
     * 
     * @param config Configuration parameters for driving behavior
     * @param cmd_vel_publisher Publisher for velocity commands
     * @param logger ROS logger for output messages
     * @param clock ROS clock for time tracking
     */
    DriveStraightBehavior(
        Config config,
        rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_publisher,
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock);

    /**
     * @brief Default destructor
     */
    ~DriveStraightBehavior() = default;

    /**
     * @brief Execute one cycle of the drive straight behavior
     * 
     * Sends velocity commands to drive the robot forward while monitoring distance
     * traveled and checking for obstacles or dock detection.
     * 
     * @param data Current robot sensor and state data
     * @return State RUNNING if still driving, SUCCESS if target distance reached or
     *               safely stopped due to obstacle or dock
     */
    State execute(const Data & data) override;

    /**
     * @brief Get the behavior's unique identifier
     * 
     * @return int32_t DRIVE_STRAIGHT identifier from Coverage action feedback constants
     */
    int32_t get_id() const override { return create3_examples_msgs::action::Coverage::Feedback::DRIVE_STRAIGHT; }

private:
    Config m_config;  ///< Behavior configuration parameters

    bool m_first_run;  ///< Flag for first execution to capture initial position
    rclcpp::Time m_start_time;  ///< Time when behavior started executing
    geometry_msgs::msg::Point m_initial_position;  ///< Starting position for distance calculation

    rclcpp::Publisher<TwistMsg>::SharedPtr m_cmd_vel_publisher;  ///< Publisher for velocity commands
    rclcpp::Logger m_logger;  ///< Logger for messages
    rclcpp::Clock::SharedPtr m_clock;  ///< Clock for time tracking
};

} // namespace create3_straight_coverage
