// Copyright 2021 iRobot Corporation. All Rights Reserved.

#include <chrono>
#include <math.h>

#include "create3_straight_coverage/straight_coverage_state_machine.hpp"
#include "tf2/utils.h"

namespace create3_straight_coverage {

/**
 * @brief Constructor for StraightCoverageStateMachine
 *
 * Initializes a new state machine with all required resources. Sets up
 * initial state variables and stores references to the ROS communication
 * interfaces needed to control the robot.
 *
 * @param goal Coverage goal parameters
 * @param clock ROS clock for time tracking
 * @param logger Logger for outputting status messages
 * @param dock_action_client Client for sending docking actions
 * @param undock_action_client Client for sending undocking actions
 * @param cmd_vel_publisher Publisher for velocity commands
 * @param has_reflexes Whether the robot has built-in reflexes enabled
 */
StraightCoverageStateMachine::StraightCoverageStateMachine(
    create3_examples_msgs::action::Coverage::Goal goal,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Logger logger,
    rclcpp_action::Client<DockAction>::SharedPtr dock_action_client,
    rclcpp_action::Client<UndockAction>::SharedPtr undock_action_client,
    rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_publisher,
    bool has_reflexes)
    : m_logger(logger)
{
    m_goal = goal;

    m_clock = clock;
    m_start_time = m_clock->now();
    m_has_reflexes = has_reflexes;

    m_dock_action_client = dock_action_client;
    m_undock_action_client = undock_action_client;
    m_cmd_vel_publisher = cmd_vel_publisher;

    // Initialize state tracking variables
    m_undocking = false;
    m_preparing_spiral = false;
    
    // Initialize coverage output to RUNNING state
    m_coverage_output.state = State::RUNNING;
    m_coverage_output.current_behavior = -1;  // No active behavior yet
}

/**
 * @brief Destructor for StraightCoverageStateMachine
 *
 * Ensures proper cleanup of resources by calling the cancel method,
 * which will terminate any active behaviors.
 */
StraightCoverageStateMachine::~StraightCoverageStateMachine()
{
    this->cancel();
}

/**
 * @brief Execute one cycle of the state machine
 *
 * This is the main method that drives the state machine's execution.
 * When first called, it selects an initial behavior based on robot state.
 * On subsequent calls, it either continues the current behavior or selects
 * a new one based on the previous behavior's outcome.
 *
 * @param data Current sensor and state data from the robot
 * @return CoverageOutput Current state and active behavior ID
 */
StraightCoverageStateMachine::CoverageOutput StraightCoverageStateMachine::execute(const Behavior::Data& data)
{
    // First execution - need to select an initial behavior
    if (!m_current_behavior) {
        this->select_start_behavior(data);
    } else {
        // Not the first execution - manage behavior transitions
        this->select_next_behavior(data);
    }

    // If state machine has completed (SUCCESS or FAILURE), return immediately
    if (m_coverage_output.state != State::RUNNING) {
        return m_coverage_output;
    }

    // Execute one cycle of the current behavior
    m_behavior_state = m_current_behavior->execute(data);
    
    // Update the output with the current behavior ID
    m_coverage_output.current_behavior = m_current_behavior->get_id();

    return m_coverage_output;
}

/**
 * @brief Cancel the current behavior and state machine execution
 *
 * This method ensures proper cleanup when the state machine execution
 * is terminated prematurely. It calls cleanup on the active behavior
 * (if any) and resets the behavior pointer.
 */
void StraightCoverageStateMachine::cancel()
{
    if (m_current_behavior) {
        m_current_behavior->cleanup();  // Call cleanup to cancel any pending actions
        m_current_behavior.reset();     // Reset the shared pointer
    }
}

/**
 * @brief Select the initial behavior based on robot state
 *
 * Determines the first behavior to run based on the robot's current state.
 * If the robot is docked, it will begin with undocking; otherwise it will
 * start with a drive straight behavior to begin coverage.
 *
 * @param data Current sensor and state data from the robot
 */
void StraightCoverageStateMachine::select_start_behavior(const Behavior::Data& data)
{
    // If robot is docked, first behavior must be undock
    if (data.dock.is_docked) {
        this->goto_undock();
    } else {
        // Otherwise, start with driving straight
        this->goto_drive_straight(DriveStraightBehavior::Config());
    }
}

/**
 * @brief Select the next behavior to execute
 *
 * This is the core decision-making function of the state machine.
 * Based on the current behavior and its execution result, it decides
 * what behavior should run next. It implements the main state transitions
 * that define the coverage algorithm.
 *
 * @param data Current sensor and state data from the robot
 */
void StraightCoverageStateMachine::select_next_behavior(const Behavior::Data& data)
{
    // Keep going with the current behavior if it's still running
    if (m_behavior_state == State::RUNNING) {
        m_coverage_output.state = State::RUNNING;
        return;
    }

    // Check if it's time to wrap up the behavior
    // Either due to reaching explore duration or max runtime
    bool explore_duration_elapsed = m_clock->now() - m_start_time >= m_goal.explore_duration;
    bool max_runtime_elapsed = m_clock->now() - m_start_time >= m_goal.max_runtime;
    
    // If we've reached the maximum runtime, terminate with success
    if (max_runtime_elapsed) {
        m_coverage_output.state = State::SUCCESS;
        return;
    }
    
    // If explore duration is over and we can see the dock, go dock the robot
    if (m_current_behavior->get_id() != FeedbackMsg::DOCK && 
        explore_duration_elapsed && data.dock.dock_visible)
    {
        this->goto_dock();
        return;
    }

    // State transition logic based on current behavior and its outcome
    switch (m_current_behavior->get_id())
    {
        case FeedbackMsg::DOCK:
        {
            // A dock action should indicate the termination of the behavior.
            // Do not set a new behavior, either return SUCCESS or FAILURE.
            if (m_behavior_state == State::FAILURE || !data.dock.is_docked) {
                m_coverage_output.state = State::FAILURE;
                break;
            }
            m_coverage_output.state = State::SUCCESS;
            break;
        }
        case FeedbackMsg::DRIVE_STRAIGHT:
        {
            // If we just undocked, then we want to start with a straight motion
            if (m_undocking) {
                m_undocking = false;
                // Continue with straight-line motion after undocking
                this->goto_drive_straight(DriveStraightBehavior::Config());
                break;
            }

            // Usually after DRIVE_STRAIGHT, rotate to change direction
            // If the drive straight failed (hit obstacle/wall), use a randomly computed angle
            auto rotate_config = RotateBehavior::Config();
            if (m_behavior_state == State::FAILURE) {
                
                // Check if we failed too many times consecutively - terminate if stuck
                if (m_evade_attempts.size() > 20) {
                    m_coverage_output.state = State::FAILURE;
                    break;
                } 

                // Use 10-degree resolution for finding new directions
                constexpr double evade_resolution = 0.175433; // 10 degrees
                rotate_config.target_rotation = compute_evade_rotation(data.pose, evade_resolution);
            } else {
                // If drive straight succeeded, clear evade attempt history
                m_evade_attempts.clear();
            }
            
            // Transition to rotate behavior
            this->goto_rotate(rotate_config);
            break;
        }
        case FeedbackMsg::ROTATE:
        {
            // A rotate failure indicates that we haven't been able to clear hazards
            if (m_behavior_state == State::FAILURE) {
                m_coverage_output.state = State::FAILURE;
                break;
            }            

            // After successful rotation, transition back to straight-line driving
            auto drive_config = DriveStraightBehavior::Config();
            this->goto_drive_straight(drive_config);
            break;
        }
        case FeedbackMsg::UNDOCK:
        {
            if (m_behavior_state == State::FAILURE || data.dock.is_docked) {
                m_coverage_output.state = State::FAILURE;
                break;
            }

            // After undocking, drive straight to move away from the dock
            auto drive_config = DriveStraightBehavior::Config();
            drive_config.max_distance = 0.5; // Move 0.5 meters initially
            drive_config.min_distance = 0.5;
            this->goto_drive_straight(drive_config);
            break;
        }
    }
}

/**
 * @brief Compute a new rotation angle for obstacle avoidance
 *
 * This function generates a rotation angle that avoids previously tried directions
 * when the robot encounters obstacles repeatedly. It keeps track of previously
 * attempted orientations and ensures the new target orientation is significantly
 * different to prevent the robot from getting stuck in cycles.
 *
 * @param pose Current robot pose containing orientation information
 * @param resolution Minimum angular difference (in radians) between attempts
 * @return double Target rotation angle in radians
 */
double StraightCoverageStateMachine::compute_evade_rotation(const geometry_msgs::msg::Pose& pose, double resolution)
{
    // Extract current orientation as quaternion and convert to Euler angles
    tf2::Quaternion current_orientation;
    tf2::convert(pose.orientation, current_orientation);

    // Add current orientation to the list of failed attempts
    double current_yaw = tf2::getYaw(current_orientation);
    m_evade_attempts.push_back(current_yaw);

    tf2::Quaternion target_orientation;
    size_t i = 0;
    // We don't want this loop to search forever.
    // Eventually, if we failed too many times, return an orientation regardless of how different it is
    // from previous attempts.
    while (i < 100) {
        // Generate a new, random, target orientation between -π and π
        double random_num = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        double random_angle = random_num * 2 * M_PI - M_PI;
        target_orientation.setRPY(0.0, 0.0, random_angle);

        // Check if the random orientation is different enough from past evade attempts
        bool valid_target = true;
        for (double angle : m_evade_attempts) {
            // Create quaternion from previously attempted angle
            tf2::Quaternion attempt_orientation;
            attempt_orientation.setRPY(0.0, 0.0, angle);

            // Calculate relative rotation between new target and previous attempt
            tf2::Quaternion relative_orientation = target_orientation * attempt_orientation.inverse();
            double relative_yaw = tf2::getYaw(relative_orientation);
            
            // If new target is too similar to a previous attempt, reject it
            if (std::abs(relative_yaw) < std::abs(resolution)) {
                valid_target = false;
                break;
            }
        }

        // Exit as soon as we find a valid target orientation
        if (valid_target) {
            break;
        }
        i++;
    }

    // Calculate the relative rotation needed from current orientation to target orientation
    tf2::Quaternion relative_orientation = target_orientation * current_orientation.inverse();
    double relative_yaw_rotation = tf2::getYaw(relative_orientation);
    return relative_yaw_rotation;
}

void StraightCoverageStateMachine::goto_dock()
{
    m_current_behavior = std::make_unique<DockBehavior>(m_dock_action_client, m_logger);
    m_coverage_output.state = State::RUNNING;
}

void StraightCoverageStateMachine::goto_drive_straight(const DriveStraightBehavior::Config& config)
{
    m_current_behavior = std::make_shared<DriveStraightBehavior>(config, m_cmd_vel_publisher, m_logger, m_clock);
    m_coverage_output.state = State::RUNNING;
}

void StraightCoverageStateMachine::goto_rotate(const RotateBehavior::Config& config)
{
    m_current_behavior = std::make_shared<RotateBehavior>(config, m_cmd_vel_publisher, m_logger, m_clock);
    m_coverage_output.state = State::RUNNING;
}

// void StraightCoverageStateMachine::goto_spiral(const SpiralBehavior::Config& config)
// {
//     m_last_spiral_time = m_clock->now();
//     m_current_behavior = std::make_shared<SpiralBehavior>(config, m_cmd_vel_publisher, m_logger, m_clock);
//     m_coverage_output.state = State::RUNNING;
// }

void StraightCoverageStateMachine::goto_undock()
{
    m_undocking = true;
    m_current_behavior = std::make_unique<UndockBehavior>(m_undock_action_client, m_logger);
    m_coverage_output.state = State::RUNNING;
}

} // namespace create3_straight_coverage
