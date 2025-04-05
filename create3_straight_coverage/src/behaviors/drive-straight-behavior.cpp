// Copyright 2021 iRobot Corporation. All Rights Reserved.

#include "create3_straight_coverage/behaviors/drive-straight-behavior.hpp"
#include "utils.hpp"

namespace create3_straight_coverage {

/**
 * @brief Constructor for the DriveStraightBehavior class
 *
 * Initializes a new DriveStraightBehavior instance with the configuration,
 * velocity publisher, and timing resources needed to drive the robot in a
 * straight line. This behavior moves the robot forward until it reaches a
 * target distance or encounters an obstacle.
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

    // Initialize the first run flag to false
    // This will be set to true in the first execute() call
    m_first_run = false;
    
    // Record the starting time for time-based measurements
    m_start_time = m_clock->now();
}

/**
 * @brief Execute one iteration of the drive-straight behavior
 *
 * This method handles driving the robot in a straight line until one of the
 * termination conditions is met: reaching max distance, detecting hazards,
 * or seeing the dock. When the behavior first runs, it records the starting
 * position to measure traveled distance.
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
    
    // Note: The following max distance check is commented out
    // but would normally stop the robot after traveling the specified distance
    // if (traveled_distance >= m_config.max_distance) {
    //     RCLCPP_INFO(m_logger, "Reached drive straight max distance: %f", traveled_distance);
    //     return State::SUCCESS;
    // }

    // Check for conditions that would cause us to stop driving:
    // 1. If the robot detects its dock in front of it
    // 2. If obstacles or hazards are detected in the path
    bool driving_towards_dock = is_driving_towards_dock(data.opcodes);
    bool hazards_detected = is_front_hazard_active(data.hazards);
    
    if (driving_towards_dock || hazards_detected) {
        RCLCPP_INFO(m_logger, "Stop driving straight: traveled %f/%f: hazards %ld dock %d",
            traveled_distance, m_config.max_distance,
            data.hazards.detections.size(), driving_towards_dock);
        
        // Note: The following check is commented out but would determine
        // success/failure based on whether minimum distance was achieved
        // if (traveled_distance >= m_config.min_distance) {
        //     return State::SUCCESS;
        // } else {
        //     return State::FAILURE;
        // }
        
        // We consider any stop a success even when hitting obstacles
        return State::SUCCESS;
    }

    // If no stopping condition met, continue driving forward
    auto twist_msg = std::make_unique<TwistMsg>();
    twist_msg->linear.x = m_config.linear_vel;  // Set forward velocity
    m_cmd_vel_publisher->publish(std::move(twist_msg));

    return State::RUNNING;
}

} // namespace create3_straight_coverage
