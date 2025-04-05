// Copyright 2021 iRobot Corporation. All Rights Reserved.

#pragma once

#include "create3_spiral_coverage/behaviors/behavior.hpp"
#include "create3_spiral_coverage/behaviors/dock-behavior.hpp"
#include "create3_spiral_coverage/behaviors/drive-straight-behavior.hpp"
#include "create3_spiral_coverage/behaviors/rotate-behavior.hpp"
#include "create3_spiral_coverage/behaviors/spiral-behavior.hpp"
#include "create3_spiral_coverage/behaviors/undock-behavior.hpp"
#include "create3_spiral_coverage/state.hpp"
#include "create3_examples_msgs/action/coverage.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "irobot_create_msgs/action/dock.hpp"
#include "irobot_create_msgs/action/undock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace create3_spiral_coverage {

/**
 * @brief State machine for implementing the spiral coverage algorithm
 * 
 * This state machine coordinates a series of robot behaviors to implement
 * an efficient coverage algorithm that combines spiral movement patterns with
 * obstacle avoidance and navigation behaviors. It manages transitions between
 * different behaviors based on sensor inputs and coverage goals.
 */
class SpiralCoverageStateMachine
{
public:
    using DockAction = irobot_create_msgs::action::Dock;
    using UndockAction = irobot_create_msgs::action::Undock;
    using TwistMsg = geometry_msgs::msg::Twist;

    /**
     * @brief Output structure for the coverage state machine
     * 
     * Contains the current state of the coverage algorithm and
     * the identifier of the active behavior.
     */
    struct CoverageOutput
    {
        int32_t current_behavior;  ///< ID of the currently active behavior
        State state;               ///< Current state of the coverage algorithm
    };

    /**
     * @brief Constructor for the SpiralCoverageStateMachine
     * 
     * Initializes a new state machine with the given goal parameters and
     * ROS resources for controlling the robot.
     * 
     * @param goal Coverage goal parameters
     * @param clock ROS clock for time tracking
     * @param logger Logger for outputting status messages
     * @param dock_action_client Client for sending docking actions
     * @param undock_action_client Client for sending undocking actions
     * @param cmd_vel_publisher Publisher for velocity commands
     * @param has_reflexes Whether the robot has built-in reflexes enabled
     */
    SpiralCoverageStateMachine(
        create3_examples_msgs::action::Coverage::Goal goal,
        rclcpp::Clock::SharedPtr clock,
        rclcpp::Logger logger,
        rclcpp_action::Client<DockAction>::SharedPtr dock_action_client,
        rclcpp_action::Client<UndockAction>::SharedPtr undock_action_client,
        rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_publisher,
        bool has_reflexes);

    /**
     * @brief Destructor
     * 
     * Ensures proper cleanup of active behaviors.
     */
    ~SpiralCoverageStateMachine();

    /**
     * @brief Execute one cycle of the state machine
     * 
     * Processes the current sensor data, determines the appropriate behavior,
     * and executes one cycle of that behavior.
     * 
     * @param data Current robot sensor and state data
     * @return CoverageOutput The current state of the coverage algorithm
     */
    CoverageOutput execute(const Behavior::Data& data);

    /**
     * @brief Cancel the current behavior and state machine execution
     * 
     * Ensures proper cleanup of resources when the state machine's execution
     * is interrupted.
     */
    void cancel();

private:
    using FeedbackMsg = create3_examples_msgs::action::Coverage::Feedback;

    /**
     * @brief Select the initial behavior based on robot state
     * 
     * Determines the first behavior to run based on the robot's current state,
     * particularly whether it's docked or undocked.
     * 
     * @param data Current robot sensor and state data
     */
    void select_start_behavior(const Behavior::Data& data);

    /**
     * @brief Select the next behavior to execute
     * 
     * Based on the result of the current behavior and the robot's state,
     * determines the next behavior to execute or terminates the state machine.
     * 
     * @param data Current robot sensor and state data
     */
    void select_next_behavior(const Behavior::Data& data);

    /**
     * @brief Transition to the dock behavior
     * 
     * Sets up and activates the docking behavior.
     */
    void goto_dock();

    /**
     * @brief Transition to the drive straight behavior
     * 
     * Sets up and activates the drive straight behavior with optional configuration.
     * 
     * @param config Configuration parameters for the drive straight behavior
     */
    void goto_drive_straight(const DriveStraightBehavior::Config& config = DriveStraightBehavior::Config());

    /**
     * @brief Transition to the rotate behavior
     * 
     * Sets up and activates the rotate behavior with optional configuration.
     * 
     * @param config Configuration parameters for the rotate behavior
     */
    void goto_rotate(const RotateBehavior::Config& config = RotateBehavior::Config());

    /**
     * @brief Transition to the spiral behavior
     * 
     * Sets up and activates the spiral behavior with optional configuration.
     * 
     * @param config Configuration parameters for the spiral behavior
     */
    void goto_spiral(const SpiralBehavior::Config& config = SpiralBehavior::Config());

    /**
     * @brief Transition to the undock behavior
     * 
     * Sets up and activates the undocking behavior.
     */
    void goto_undock();

    /**
     * @brief Compute a new rotation angle for obstacle avoidance
     * 
     * Generates a rotation angle that avoids previously tried directions
     * when the robot encounters obstacles repeatedly.
     * 
     * @param pose Current robot pose
     * @param resolution Minimum angular difference between attempts
     * @return double Target rotation angle in radians
     */
    double compute_evade_rotation(const geometry_msgs::msg::Pose& pose, double resolution);

    std::shared_ptr<Behavior> m_current_behavior;  ///< Currently active behavior
    State m_behavior_state;  ///< State of the current behavior

    bool m_undocking;  ///< Flag indicating if we're in the undocking process
    rclcpp::Time m_last_spiral_time;  ///< Time when spiral behavior was last active
    bool m_preparing_spiral;  ///< Flag indicating if we're preparing to spiral
    std::vector<double> m_evade_attempts;  ///< History of attempted evade angles

    CoverageOutput m_coverage_output;  ///< Current output state of the state machine
    create3_examples_msgs::action::Coverage::Goal m_goal;  ///< Active coverage goal
    rclcpp::Time m_start_time;  ///< Time when the state machine started
    bool m_has_reflexes;  ///< Whether the robot has built-in reflexes

    rclcpp_action::Client<DockAction>::SharedPtr m_dock_action_client;  ///< Client for docking
    rclcpp_action::Client<UndockAction>::SharedPtr m_undock_action_client;  ///< Client for undocking
    rclcpp::Publisher<TwistMsg>::SharedPtr m_cmd_vel_publisher;  ///< Publisher for velocity commands
    rclcpp::Logger m_logger;  ///< Logger for status messages
    rclcpp::Clock::SharedPtr m_clock;  ///< Clock for time tracking
};

} // namespace create3_spiral_coverage
