// Copyright 2021 iRobot Corporation. All Rights Reserved.

#include "create3_straight_coverage/behaviors/dock-behavior.hpp"

namespace create3_straight_coverage {

/**
 * @brief Constructor for the DockBehavior class
 *
 * Initializes a new DockBehavior instance with the necessary action client 
 * and logger for handling robot docking operations. The dock behavior manages
 * communication with the robot's docking action server to safely guide the
 * robot back to its charging station.
 *
 * @param dock_action_client Action client used to send docking goals to the robot
 * @param logger Logger instance for outputting status and debug information
 */
DockBehavior::DockBehavior(
    rclcpp_action::Client<DockAction>::SharedPtr dock_action_client,
    rclcpp::Logger logger)
: m_dock_action_client(dock_action_client), m_logger(logger)
{
    // No additional initialization needed
}

/**
 * @brief Execute one iteration of the docking behavior
 *
 * This method implements a state machine for docking the robot. It first checks
 * if the robot is already docked (in which case it fails), then waits for the
 * action server to be ready, sends the docking command, and monitors progress.
 * The method should be called repeatedly until it returns SUCCESS or FAILURE.
 *
 * @param data Current sensor and state data from the robot
 * @return State Indicates the current execution state:
 *         - RUNNING: Docking process is ongoing
 *         - SUCCESS: Robot has successfully docked
 *         - FAILURE: Docking attempt has failed (e.g., already docked, goal rejected)
 */
State DockBehavior::execute(const Data & data)
{
    // Make sure we are not already docked - no point in trying to dock if docked
    if (!m_dock_action_sent && data.dock.is_docked) {
        RCLCPP_ERROR(m_logger, "Robot is already docked!");
        return State::FAILURE;
    }

    // We can't dock until we discover the docking action server
    // This check prevents sending commands to a non-existent server
    if (!m_dock_action_client->action_server_is_ready()) {
        RCLCPP_DEBUG(m_logger, "Waiting for dock action server");
        return State::RUNNING;
    }

    // Send dock command if not already sent and if we are not waiting for result
    if (!m_dock_action_sent) {
        RCLCPP_INFO(m_logger, "Sending docking goal!");
        auto goal_msg = DockAction::Goal();

        // Configure the callbacks for handling asynchronous action responses
        auto send_goal_options = rclcpp_action::Client<DockAction>::SendGoalOptions();
        
        // This callback is triggered when the goal is accepted or rejected by the server
        send_goal_options.goal_response_callback = [this](const GoalHandleDock::SharedPtr & goal_handle){
            m_dock_goal_handle_ready = true;
            m_dock_goal_handle = goal_handle;
        };
        
        // This callback is triggered when the action completes (successfully or not)
        send_goal_options.result_callback = [this](const GoalHandleDock::WrappedResult & result){
            m_dock_result_ready = true;
            m_dock_result = result;
        };

        // Send the goal asynchronously - execution continues while docking happens
        m_dock_action_client->async_send_goal(goal_msg, send_goal_options);
        m_dock_action_sent = true;

        return State::RUNNING;
    }

    // Check if goal was rejected by the server
    // This happens if the goal handle is ready but the handle itself is null
    if (m_dock_goal_handle_ready && !m_dock_goal_handle) {
        RCLCPP_ERROR(m_logger, "Docking goal was rejected by server");
        return State::FAILURE;
    }

    // Check if we have received a result from the docking action
    if (m_dock_result_ready) {
        if (m_dock_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(m_logger, "Docking succeeded!");
            return State::SUCCESS;
        } else {
            RCLCPP_ERROR(m_logger, "Docking failed!");
            return State::FAILURE;
        }
    }

    // Action is still in progress
    return State::RUNNING;
}

/**
 * @brief Clean up resources when the behavior is cancelled
 *
 * This method handles proper cancellation of the docking behavior.
 * If a docking action is in progress but not yet completed, it sends
 * a cancellation request to the action server to stop the docking process.
 * This ensures the robot stops attempting to dock if the behavior is
 * interrupted.
 */
void DockBehavior::cleanup()
{
    // This behavior is being cancelled, so send a cancel request to dock action server if it's running
    // We only cancel if:
    // 1. We don't have a result yet (m_dock_result_ready is false)
    // 2. We have a valid goal handle (m_dock_goal_handle_ready is true and m_dock_goal_handle is valid)
    if (!m_dock_result_ready && m_dock_goal_handle_ready && m_dock_goal_handle) {
        m_dock_action_client->async_cancel_goal(m_dock_goal_handle);
    }
}

} // namespace create3_straight_coverage
