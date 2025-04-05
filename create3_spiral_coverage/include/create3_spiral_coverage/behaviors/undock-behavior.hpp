// Copyright 2021 iRobot Corporation. All Rights Reserved.

#pragma once

#include "create3_spiral_coverage/behaviors/behavior.hpp"
#include "create3_examples_msgs/action/coverage.hpp"
#include "irobot_create_msgs/action/undock.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace create3_spiral_coverage {

/**
 * @brief Behavior that handles the robot's undocking procedure
 * 
 * This behavior sends an undocking action goal to the robot and monitors its progress.
 * It manages the asynchronous action communication with the undocking action server,
 * including handling goal acceptance, rejection, and completion.
 */
class UndockBehavior : public Behavior
{
public:
    using UndockAction = irobot_create_msgs::action::Undock;  ///< The undocking action type
    using GoalHandleUndock = rclcpp_action::ClientGoalHandle<UndockAction>;  ///< Handle type for the action goal

    /**
     * @brief Constructor for UndockBehavior
     * 
     * Initializes the undocking behavior with an action client for sending undocking goals
     * and a logger for outputting status information.
     * 
     * @param undock_action_client Client to send undocking goals to the robot
     * @param logger ROS logger for status and error messages
     */
    UndockBehavior(
        rclcpp_action::Client<UndockAction>::SharedPtr undock_action_client,
        rclcpp::Logger logger);

    /**
     * @brief Default destructor
     */
    ~UndockBehavior() = default;

    /**
     * @brief Execute one cycle of the undocking behavior
     * 
     * Implements the undocking state machine. First check if the robot is docked,
     * then wait for action server, send undocking goal, and finally monitor progress.
     * 
     * @param data Current robot sensor and state data
     * @return State RUNNING if undocking in progress, SUCCESS if undocked successfully,
     *               FAILURE if undocking failed or robot already undocked
     */
    State execute(const Data & data) override;

    /**
     * @brief Get the behavior's unique identifier
     * 
     * @return int32_t UNDOCK identifier from Coverage action feedback constants
     */
    int32_t get_id() const override { return create3_examples_msgs::action::Coverage::Feedback::UNDOCK; }

    /**
     * @brief Cancel any ongoing undocking actions when behavior is terminated
     * 
     * Sends a cancel request to the undocking action server if a goal is active.
     */
    void cleanup() override;

private:
    bool m_undock_action_sent {false};  ///< Flag indicating if undock goal has been sent
    GoalHandleUndock::SharedPtr m_undock_goal_handle;  ///< Handle to active undocking goal
    bool m_undock_goal_handle_ready {false};  ///< Flag indicating if goal handle has been received
    GoalHandleUndock::WrappedResult m_undock_result;  ///< Result of the undocking action
    bool m_undock_result_ready {false};  ///< Flag indicating if the result has been received

    rclcpp_action::Client<UndockAction>::SharedPtr m_undock_action_client;  ///< Client for undocking action
    rclcpp::Logger m_logger;  ///< Logger for messages
};

} // namespace create3_spiral_coverage
