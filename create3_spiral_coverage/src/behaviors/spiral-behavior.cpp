// Copyright 2021 iRobot Corporation. All Rights Reserved.

#include "create3_spiral_coverage/behaviors/spiral-behavior.hpp"
#include "utils.hpp"

namespace create3_spiral_coverage {

/**
 * @brief Constructor for the SpiralBehavior class
 *
 * Initializes a new SpiralBehavior instance with the configuration,
 * velocity publisher, and timing resources needed to move the robot in an
 * expanding spiral pattern for efficient area coverage.
 *
 * @param config Configuration parameters including spiral radius, duration, and velocity settings
 * @param cmd_vel_publisher Publisher for sending velocity commands to the robot
 * @param logger Logger instance for outputting status and debug information
 * @param clock Clock for tracking execution duration and timing for radius expansion
 */
SpiralBehavior::SpiralBehavior(
    Config config,
    rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_publisher,
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock)
: m_cmd_vel_publisher(cmd_vel_publisher), m_logger(logger)
{
    m_config = config;
    m_clock = clock;

    // Record starting time to track duration and radius updates
    m_start_time = m_clock->now();
    m_last_radius_update_time = m_start_time;
    
    // Initialize spiral radius to the configured starting value
    m_radius = m_config.initial_radius;
}

/**
 * @brief Execute one iteration of the spiral behavior
 *
 * This method handles moving the robot in an expanding spiral pattern by
 * publishing appropriate velocity commands. It periodically increases the
 * spiral radius and checks for termination conditions like obstacles or
 * dock detection.
 *
 * @param data Current sensor and state data from the robot
 * @return State Indicates the current execution state:
 *         - RUNNING: Spiral pattern is ongoing
 *         - FAILURE: Spiral terminated due to obstacle or dock detection
 */
State SpiralBehavior::execute(const Data & data)
{
    auto now = m_clock->now();
    rclcpp::Duration spiral_time = now - m_start_time;
    
    // Optional time-based termination (currently commented out)
    // if (spiral_time > m_config.spiral_duration) {
    //     RCLCPP_INFO(m_logger, "Spiral completed!");
    //     return State::SUCCESS;
    // }

    // Check if robot detects the dock or any obstacles in front
    bool driving_towards_dock = is_driving_towards_dock(data.opcodes);
    bool hazards_detected = is_front_hazard_active(data.hazards);
    
    // Stop spiraling if pointing towards dock or hazards detected
    if (driving_towards_dock || hazards_detected) {
        RCLCPP_INFO(m_logger, "Stop spiraling: time spent %f/%f hazards %ld dock %d",
        spiral_time.seconds(), m_config.spiral_duration.seconds(),
        data.hazards.detections.size(), driving_towards_dock);
        return State::FAILURE;
    }

    // Periodically increase the spiral radius according to the configured interval
    if (now - m_last_radius_update_time > m_config.radius_increment_interval) {
        m_radius += m_config.radius_increment;
        m_last_radius_update_time = now;
    }

    // Compute the proper velocity commands to create a spiral motion
    // Linear velocity is constant, angular velocity decreases as radius increases
    auto twist_msg = std::make_unique<TwistMsg>();
    twist_msg->linear.x = m_config.linear_vel;
    twist_msg->angular.z = m_config.linear_vel / m_radius;  // Angular velocity = linear_vel/radius

    RCLCPP_DEBUG(m_logger, "Spiral velocities: linear %f angular %f",
        twist_msg->linear.x, twist_msg->angular.z);

    // Send the computed velocity command to drive the robot
    m_cmd_vel_publisher->publish(std::move(twist_msg));

    return State::RUNNING;
}

} // namespace create3_spiral_coverage
