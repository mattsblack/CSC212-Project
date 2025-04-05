// Copyright 2021 iRobot Corporation. All Rights Reserved.

#pragma once

#include "create3_spiral_coverage/behaviors/behavior.hpp"
#include "create3_examples_msgs/action/coverage.hpp"
#include "irobot_create_msgs/action/dock.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace create3_spiral_coverage {

/**
 * @brief Behavior that handles the robot's docking procedure
 * 
 * This behavior sends a docking action goal to the robot and monitors its progress.
 * It manages the asynchronous action communication with the docking action server,
 * including handling goal acceptance, rejection, and completion.
 */
class DockBehavior : public Behavior
{
public:
    using DockAction = irobot_create_msgs::action::Dock;  ///< The docking action type
    using GoalHandleDock = rclcpp_action::ClientGoalHandle<DockAction>;  ///< Handle type for the action goal

    /**
     * @brief Constructor for DockBehavior
     * 
     * Initializes the docking behavior with an action client for sending docking goals
     * and a logger for outputting status information.
     * 
     * @param dock_action_client Client to send docking goals to the robot
     * @param logger ROS logger for status and error messages
     */
    DockBehavior(
        rclcpp_action::Client<DockAction>::SharedPtr dock_action_client,
        rclcpp::Logger logger);

    /**
     * @brief Default destructor
     */
    ~DockBehavior() = default;

    /**
     * @brief Execute one cycle of the docking behavior
     * 
     * Implements the docking state machine. First check if the robot is already docked,
     * then wait for action server, send docking goal, and finally monitor progress.
     * 
     * @param data Current robot sensor and state data
     * @return State RUNNING if docking in progress, SUCCESS if docked successfully, 
     *               FAILURE if docking failed or robot already docked
     */
    State execute(const Data & data) override;

    /**
     * @brief Get the behavior's unique identifier
     * 
     * @return int32_t DOCK identifier from Coverage action feedback constants
     */
    int32_t get_id() const override { return create3_examples_msgs::action::Coverage::Feedback::DOCK; }

    /**
     * @brief Cancel any ongoing docking actions when behavior is terminated
     * 
     * Sends a cancel request to the docking action server if a goal is active.
     */
    void cleanup() override;

private:
    bool m_dock_action_sent {false};  ///< Flag indicating if dock goal has been sent
    GoalHandleDock::SharedPtr m_dock_goal_handle;  ///< Handle to active docking goal
    bool m_dock_goal_handle_ready {false};  ///< Flag indicating if goal handle has been received
    GoalHandleDock::WrappedResult m_dock_result;  ///< Result of the docking action
    bool m_dock_result_ready {false};  ///< Flag indicating if the result has been received

    rclcpp_action::Client<DockAction>::SharedPtr m_dock_action_client;  ///< Client for docking action
    rclcpp::Logger m_logger;  ///< Logger for messages
};

} // namespace create3_spiral_coverage
