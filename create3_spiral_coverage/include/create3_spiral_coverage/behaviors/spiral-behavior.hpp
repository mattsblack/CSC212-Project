// Copyright 2021 iRobot Corporation. All Rights Reserved.

#pragma once

#include "create3_spiral_coverage/behaviors/behavior.hpp"
#include "create3_examples_msgs/action/coverage.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace create3_spiral_coverage {

/**
 * @brief Behavior that moves the robot in an expanding spiral pattern
 * 
 * This behavior commands the robot to move in a spiral pattern by combining
 * linear and angular velocities. The spiral expands over time by increasing
 * the radius, providing efficient area coverage.
 */
class SpiralBehavior : public Behavior
{
public:
    using OpCodeMsg = irobot_create_msgs::msg::IrOpcode;  ///< Type for IR sensor opcodes
    using TwistMsg = geometry_msgs::msg::Twist;  ///< Type for velocity commands

    /**
     * @brief Configuration parameters for the spiral behavior
     */
    struct Config
    {
        rclcpp::Duration spiral_duration {rclcpp::Duration(std::chrono::seconds(30))};  ///< Maximum duration for spiraling
        double linear_vel {0.3};  ///< Linear velocity in meters per second
        double initial_radius {0.25};  ///< Starting radius of the spiral in meters
        double radius_increment {0.25};  ///< Amount to increase radius by each interval
        rclcpp::Duration radius_increment_interval {rclcpp::Duration(std::chrono::seconds(5))};  ///< Time between radius increases
    };

    /**
     * @brief Constructor for SpiralBehavior
     * 
     * @param config Configuration parameters for spiraling
     * @param cmd_vel_publisher Publisher for velocity commands
     * @param logger ROS logger for output messages
     * @param clock ROS clock for time tracking
     */
    SpiralBehavior(
        Config config,
        rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_publisher,
        rclcpp::Logger logger,
        rclcpp::Clock::SharedPtr clock);

    /**
     * @brief Default destructor
     */
    ~SpiralBehavior() = default;

    /**
     * @brief Execute one cycle of the spiral behavior
     * 
     * Sends velocity commands to move the robot in a spiral pattern while
     * periodically increasing the radius. Monitors for obstacles and dock detection.
     * 
     * @param data Current robot sensor and state data
     * @return State RUNNING if still spiraling, FAILURE if stopped due to obstacle or dock
     */
    State execute(const Data & data) override;

    /**
     * @brief Get the behavior's unique identifier
     * 
     * @return int32_t SPIRAL identifier from Coverage action feedback constants
     */
    int32_t get_id() const override { return create3_examples_msgs::action::Coverage::Feedback::SPIRAL; }

private:
    rclcpp::Time m_start_time;  ///< Time when behavior started executing
    rclcpp::Time m_last_radius_update_time;  ///< Time when radius was last increased
    double m_radius;  ///< Current spiral radius

    Config m_config;  ///< Behavior configuration parameters

    rclcpp::Publisher<TwistMsg>::SharedPtr m_cmd_vel_publisher;  ///< Publisher for velocity commands
    rclcpp::Logger m_logger;  ///< Logger for messages
    rclcpp::Clock::SharedPtr m_clock;  ///< Clock for time tracking
};

} // namespace create3_spiral_coverage
