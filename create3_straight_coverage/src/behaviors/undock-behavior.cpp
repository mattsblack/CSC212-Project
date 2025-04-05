// Copyright 2021 iRobot Corporation. All Rights Reserved.

#include "create3_straight_coverage/behaviors/undock-behavior.hpp"

namespace create3_straight_coverage {

/**
 * @brief Constructor for the UndockBehavior class
 *
 * Initializes a new UndockBehavior instance with the necessary action client 
 * and logger for handling robot undocking operations. The undock behavior
 * manages communication with the robot's undocking action server to safely
 * detach the robot from its charging station.
 *
 * @param undock_action_client Action client used to send undocking goals to the robot
 * @param logger Logger instance for outputting status and debug information
 */
UndockBehavior::UndockBehavior(
    rclcpp_action::Client<UndockAction>::SharedPtr undock_action_client,
    rclcpp::Logger logger)
: m_undock_action_client(undock_action_client), m_logger(logger)
{
    // No additional initialization needed
}

/**
 * @brief Execute one iteration of the undocking behavior
 *
 * This method implements a state machine for undocking the robot. It first checks
 * if the robot is already undocked (in which case it fails), then waits for the
 * action server to be ready, sends the undocking command, and monitors progress.
 * The method should be called repeatedly until it returns SUCCESS or FAILURE.
 *
 * @param data Current sensor and state data from the robot
 * @return State Indicates the current execution state:
 *         - RUNNING: Undocking process is ongoing
 *         - SUCCESS: Robot has successfully undocked
 *         - FAILURE: Undocking attempt has failed (e.g., already undocked, goal rejected)
 */
State UndockBehavior::execute(const Data & data)
{
    // Make sure we are actually docked - no point in trying to undock if already undocked
    if (!m_undock_action_sent && !data.dock.is_docked) {
        RCLCPP_ERROR(m_logger, "Robot is already undocked!");
        return State::FAILURE;
    }

    // We can't undock until we discover the undocking action server
    // This check prevents sending commands to a non-existent server
    if (!m_undock_action_client->action_server_is_ready()) {
        RCLCPP_DEBUG(m_logger, "Waiting for undock action server");
        return State::RUNNING;
    }

    // Send undock command if not already sent and if we are not waiting for result
    if (!m_undock_action_sent) {
        RCLCPP_INFO(m_logger, "Sending undocking goal!");
        auto goal_msg = UndockAction::Goal();

        // Configure the callbacks for handling asynchronous action responses
        auto send_goal_options = rclcpp_action::Client<UndockAction>::SendGoalOptions();
        
        // This callback is triggered when the goal is accepted or rejected by the server
        send_goal_options.goal_response_callback = [this](const GoalHandleUndock::SharedPtr & goal_handle){
            m_undock_goal_handle_ready = true;
            m_undock_goal_handle = goal_handle;
        };
        
        // This callback is triggered when the action completes (successfully or not)
        send_goal_options.result_callback = [this](const GoalHandleUndock::WrappedResult & result){
            m_undock_result_ready = true;
            m_undock_result = result;
        };

        // Send the goal asynchronously - execution continues while undocking happens
        m_undock_action_client->async_send_goal(goal_msg, send_goal_options);
        m_undock_action_sent = true;

        return State::RUNNING;
    }

    // Check if goal was rejected by the server
    // This happens if the goal handle is ready but the handle itself is null
    if (m_undock_goal_handle_ready && !m_undock_goal_handle) {
        RCLCPP_ERROR(m_logger, "Undock goal was rejected by server");
        return State::FAILURE;
    }

    // Check if we have received a result from the undocking action
    if (m_undock_result_ready) {
        if (m_undock_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(m_logger, "Undocking succeeded!");
            return State::SUCCESS;
        } else {
            RCLCPP_ERROR(m_logger, "Undocking failed!");
            return State::FAILURE;
        }
    }

    // Action is still in progress
    return State::RUNNING;
}

/**
 * @brief Clean up resources when the behavior is cancelled
 *
 * This method handles proper cancellation of the undocking behavior.
 * If an undocking action is in progress but not yet completed, it sends
 * a cancellation request to the action server to stop the undocking process.
 * This ensures the robot stops attempting to undock if the behavior is
 * interrupted.
 */
void UndockBehavior::cleanup()
{
    // This behavior is being cancelled, so send a cancel request to undock action server if it's running
    // We only cancel if:
    // 1. We don't have a result yet (m_undock_result_ready is false)
    // 2. We have a valid goal handle (m_undock_goal_handle_ready is true and m_undock_goal_handle is valid)
    if (!m_undock_result_ready && m_undock_goal_handle_ready && m_undock_goal_handle) {
        m_undock_action_client->async_cancel_goal(m_undock_goal_handle);
    }
}

} // namespace create3_straight_coverage
