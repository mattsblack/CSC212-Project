// Copyright 2021 iRobot Corporation. All Rights Reserved.

#include "create3_spiral_coverage/behaviors/reflex-behavior.hpp"
#include "utils.hpp"

namespace create3_spiral_coverage {

/**
 * @brief Constructor for the ReflexBehavior class
 *
 * Initializes a new ReflexBehavior instance that handles immediate responses
 * to hazards by backing up the robot to a safe position.
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

    // Flag to track if this is the first execution of the behavior
    m_first_run = false;
    m_start_time = m_clock->now();
}

/**
 * @brief Execute one iteration of the reflex behavior
 *
 * This method handles the immediate response to detected hazards by backing
 * up the robot a short distance. It monitors the traveled distance and
 * checks if hazards are cleared or if backup limits are reached.
 *
 * @param data Current sensor and state data from the robot
 * @return State Indicates the current execution state:
 *         - RUNNING: Backing up is still in progress
 *         - SUCCESS: Hazard has been cleared by backing up
 *         - FAILURE: Unable to clear hazard even after backing up
 */
State ReflexBehavior::execute(const Data & data)
{
    // On first execution, record starting position and check if reflex is needed
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
    bool timeout = m_clock->now() - m_start_time > m_config.clear_hazard_time;
    bool moved_enough = get_distance(data.pose.position, m_initial_position) > m_config.backup_distance;
    bool limit_reached = backup_limit_reached(data.hazards);

    // Stop backing up if we've moved enough or hit a backup limit
    if (moved_enough || limit_reached) { // timeout ||
        // Check if the hazard has been cleared
        if (is_front_hazard_active(data.hazards)) {
            RCLCPP_INFO(m_logger, "Reflex failed: was not able to clear hazard (timeout %d distance %d backup %d)",
                timeout, moved_enough, limit_reached);
            return State::FAILURE;
        } else {
            RCLCPP_INFO(m_logger, "Reflex successfully cleared hazard");
            return State::SUCCESS;
        }
    }

    // Continue backing up by publishing negative velocity command
    auto twist_msg = std::make_unique<TwistMsg>();
    twist_msg->linear.x = -m_config.linear_vel;  // Negative velocity for backing up
    m_cmd_vel_publisher->publish(std::move(twist_msg));

    return State::RUNNING;
}

/**
 * @brief Check if a backup limit hazard is detected
 *
 * Examines the hazard detections to determine if the robot has reached
 * a backup limit, which would indicate it cannot back up any further.
 *
 * @param hazards The detected hazards from the robot's sensors
 * @return bool True if a BACKUP_LIMIT hazard is detected, false otherwise
 */
bool ReflexBehavior::backup_limit_reached(const irobot_create_msgs::msg::HazardDetectionVector& hazards)
{
    // Search through all hazard detections for a BACKUP_LIMIT type
    auto limit_hazard = std::find_if(hazards.detections.begin(), hazards.detections.end(),
        [](const irobot_create_msgs::msg::HazardDetection& detection){
            return (detection.type == irobot_create_msgs::msg::HazardDetection::BACKUP_LIMIT);
        });
    
    // Return true if a backup limit hazard was found
    return limit_hazard != hazards.detections.end();
}

} // namespace create3_spiral_coverage
