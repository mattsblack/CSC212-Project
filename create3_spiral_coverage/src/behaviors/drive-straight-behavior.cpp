// Copyright 2021 iRobot Corporation. All Rights Reserved.

#include "create3_spiral_coverage/behaviors/drive-straight-behavior.hpp"
#include "utils.hpp"

namespace create3_spiral_coverage {

/**
 * @brief Constructor for the DriveStraightBehavior class
 *
 * Initializes a new DriveStraightBehavior instance with the configuration,
 * velocity publisher, and timing resources needed to drive the robot in a
 * straight line.
 *
 * @param config Configuration parameters including max distance and linear velocity
 * @param cmd_vel_publisher Publisher for sending velocity commands to the robot
 * @param logger Logger instance for outputting status and debug information
 * @param clock Clock for tracking execution duration and timing
 */
DriveStraightBehavior::DriveStraightBehavior(
    Config config,
    rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_publisher,
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock)
: m_cmd_vel_publisher(cmd_vel_publisher), m_logger(logger)
{
    m_clock = clock;
    m_config = config;

    // Flag to track if this is the first execution of the behavior
    m_first_run = false;
    m_start_time = m_clock->now();
}

/**
 * @brief Execute one iteration of the drive-straight behavior
 *
 * This method handles driving the robot in a straight line until one of the
 * termination conditions is met: reaching max distance, detecting hazards,
 * or seeing the dock.
 *
 * @param data Current sensor and state data from the robot
 * @return State Indicates the current execution state:
 *         - RUNNING: Robot is still driving forward
 *         - SUCCESS: Robot has reached target distance or safely stopped
 */
State DriveStraightBehavior::execute(const Data & data)
{
    // On first execution, record the starting position to measure distance traveled
    if (!m_first_run) {
        m_first_run = true;
        m_initial_position = data.pose.position;
    }

    // Calculate distance traveled since behavior started
    double traveled_distance = get_distance(m_initial_position, data.pose.position);
    
    // Check if we've reached the configured maximum travel distance
    if (traveled_distance >= m_config.max_distance) {
        RCLCPP_INFO(m_logger, "Reached drive straight max distance: %f", traveled_distance);
        return State::SUCCESS;
    }

    // Check if robot detects the dock or any obstacles in front
    bool driving_towards_dock = is_driving_towards_dock(data.opcodes);
    bool hazards_detected = is_front_hazard_active(data.hazards);
    
    // Stop if pointing towards dock or hazards detected
    if (driving_towards_dock || hazards_detected) {
        RCLCPP_INFO(m_logger, "Stop driving straight: traveled %f/%f: hazards %ld dock %d",
            traveled_distance, m_config.max_distance,
            data.hazards.detections.size(), driving_towards_dock);
        
        // We consider this a success even when stopping early due to obstacles or dock
        // This is a design choice - could also be configured to return FAILURE if 
        // the robot didn't travel the minimum expected distance
        return State::SUCCESS;
    }

    // Continue driving forward by publishing velocity command
    auto twist_msg = std::make_unique<TwistMsg>();
    twist_msg->linear.x = m_config.linear_vel;  // Forward velocity from config
    m_cmd_vel_publisher->publish(std::move(twist_msg));

    return State::RUNNING;
}

} // namespace create3_spiral_coverage
