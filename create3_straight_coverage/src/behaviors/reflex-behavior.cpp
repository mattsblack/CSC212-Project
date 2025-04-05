// Copyright 2021 iRobot Corporation. All Rights Reserved.

#include "create3_straight_coverage/behaviors/reflex-behavior.hpp"
#include "utils.hpp"

namespace create3_straight_coverage {

/**
 * @brief Constructor for the ReflexBehavior class
 *
 * Initializes a new ReflexBehavior instance that handles immediate responses
 * to hazards by backing up the robot to a safe position. This behavior is typically
 * used by other behaviors when they detect hazards that prevent normal operation.
 *
 * @param config Configuration parameters including backup distance and velocity
 * @param cmd_vel_publisher Publisher for sending velocity commands to the robot
 * @param logger Logger instance for outputting status and debug information
 * @param clock Clock for tracking execution duration
 */
ReflexBehavior::ReflexBehavior(
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
    
    // Record the starting time for timeout calculations
    m_start_time = m_clock->now();
}

/**
 * @brief Execute one iteration of the reflex behavior
 *
 * This method handles the immediate response to detected hazards by backing
 * up the robot a short distance. On first execution, it records the starting
 * position and checks if reflexes are actually needed. Then it monitors for
 * timeout conditions or backup limits before stopping.
 *
 * @param data Current sensor and state data from the robot
 * @return State Indicates the current execution state:
 *         - RUNNING: Backing up is still in progress
 *         - SUCCESS: Hazard has been cleared by backing up
 *         - FAILURE: Unable to clear hazard even after backing up
 */
State ReflexBehavior::execute(const Data & data)
{
    // On first execution, set up initial state and check if reflex is needed
    if (!m_first_run) {
        m_first_run = true;
        m_initial_position = data.pose.position;

        // If no hazards are detected, we don't need to run the reflex behavior
        if (!is_front_hazard_active(data.hazards)) {
            RCLCPP_INFO(m_logger, "No need to run reflex");
            return State::SUCCESS;
        }
    }

    // Check termination conditions for the backing up motion
    
    // Time-based termination - stop if we've been trying too long
    bool timeout = m_clock->now() - m_start_time > m_config.clear_hazard_time;
    
    // Note: Distance-based termination is commented out
    // bool moved_enough = get_distance(data.pose.position, m_initial_position) > m_config.backup_distance;
    
    // Obstacle-based termination - stop if we hit something while backing up
    bool limit_reached = backup_limit_reached(data.hazards);

    // Decide whether to stop backing up
    // Note: The original distance check is commented out
    // if (timeout || moved_enough || limit_reached) {
    if (timeout || limit_reached) {
        // Check if backing up cleared the hazard
        if (is_front_hazard_active(data.hazards)) {
            RCLCPP_INFO(m_logger, "Reflex failed: was not able to clear hazard (timeout %d backup %d)",
                timeout, limit_reached);
            return State::FAILURE;
        } else {
            RCLCPP_INFO(m_logger, "Reflex successfully cleared hazard");
            return State::SUCCESS;
        }
    }

    // Continue backing up by publishing negative velocity command
    auto twist_msg = std::make_unique<TwistMsg>();
    twist_msg->linear.x = -m_config.linear_vel;  // Negative value for reverse motion
    m_cmd_vel_publisher->publish(std::move(twist_msg));

    return State::RUNNING;
}

/**
 * @brief Check if a backup limit hazard is detected
 *
 * This method examines the hazard detections to determine if the robot has reached
 * a backup limit, which would indicate it cannot back up any further (such as when 
 * hitting an obstacle while reversing).
 *
 * @param hazards The detected hazards from the robot's sensors
 * @return bool True if a BACKUP_LIMIT hazard is detected, false otherwise
 */
bool ReflexBehavior::backup_limit_reached(const irobot_create_msgs::msg::HazardDetectionVector& hazards)
{
    // Search through all hazard detections for a BACKUP_LIMIT type
    // Using STL algorithm find_if for clarity and efficiency
    auto limit_hazard = std::find_if(hazards.detections.begin(), hazards.detections.end(),
        [](const irobot_create_msgs::msg::HazardDetection& detection){
            return (detection.type == irobot_create_msgs::msg::HazardDetection::BACKUP_LIMIT);
        });
    
    // Return true if a backup limit hazard was found
    return limit_hazard != hazards.detections.end();
}

} // namespace create3_straight_coverage
