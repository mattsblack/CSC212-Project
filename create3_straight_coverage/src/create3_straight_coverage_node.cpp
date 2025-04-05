// Copyright 2021 iRobot Corporation. All Rights Reserved.

#include "create3_straight_coverage/straight_coverage_state_machine.hpp"
#include "create3_straight_coverage/create3_straight_coverage_node.hpp"

#include "behaviors/utils.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace create3_straight_coverage {

Create3StraightCoverageNode::Create3StraightCoverageNode()
: rclcpp::Node("create3_straight_coverage")
{
    m_coverage_action_server = rclcpp_action::create_server<CoverageAction>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "coverage",
        std::bind(&Create3StraightCoverageNode::handle_goal, this, _1, _2),
        std::bind(&Create3StraightCoverageNode::handle_cancel, this, _1),
        std::bind(&Create3StraightCoverageNode::handle_accepted, this, _1));

    m_dock_action_client = rclcpp_action::create_client<DockAction>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "dock");

    m_undock_action_client = rclcpp_action::create_client<UndockAction>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "undock");
    
    m_cmd_vel_publisher = this->create_publisher<TwistMsg>("cmd_vel", 10);

    m_reflexes_param_client = std::make_shared<rclcpp::AsyncParametersClient>(
        this->get_node_base_interface(),
        this->get_node_topics_interface(),
        this->get_node_graph_interface(),
        this->get_node_services_interface(),
        "motion_control",
        rmw_qos_profile_parameters);

    m_dock_subscription = this->create_subscription<DockMsg>(
        "dock_status",
        rclcpp::SensorDataQoS(),
        std::bind(&Create3StraightCoverageNode::dock_callback, this, _1));

    m_hazards_subscription = this->create_subscription<HazardMsg>(
        "hazard_detection",
        rclcpp::SensorDataQoS(),
        std::bind(&Create3StraightCoverageNode::hazards_callback, this, _1));

    m_ir_opcode_subscription = this->create_subscription<OpCodeMsg>(
        "ir_opcode",
        rclcpp::SensorDataQoS(),
        std::bind(&Create3StraightCoverageNode::ir_opcode_callback, this, _1));

    m_odom_subscription = this->create_subscription<OdometryMsg>(
        "odom",
        rclcpp::SensorDataQoS(),
        std::bind(&Create3StraightCoverageNode::odom_callback, this, _1));

    m_kidnap_subscription = this->create_subscription<KidnapMsg>(
        "kidnap_status",
        rclcpp::SensorDataQoS(),
        std::bind(&Create3StraightCoverageNode::kidnap_callback, this, _1));

    m_rate_hz = this->declare_parameter<double>("rate_hz", 30);
    m_opcodes_buffer_ms = this->declare_parameter<int>("opcodes_buffer_ms", 200);

    m_dock_msgs_received = false;
    m_is_running = false;
    m_last_behavior = -1;
    m_last_opcodes_cleared_time = this->now();
    RCLCPP_INFO(this->get_logger(), "Node created!");
}

rclcpp_action::GoalResponse
Create3StraightCoverageNode::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const CoverageAction::Goal> goal)
{
    (void)uuid;
    (void)goal;

    if (!this->ready_to_start()) {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal request: robot nodes have not been discovered yet");
        return rclcpp_action::GoalResponse::REJECT;
    }

    bool is_kidnapped = false;
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        is_kidnapped = m_last_kidnap.is_kidnapped;
    }
    if (is_kidnapped) {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal request: robot is currently kidnapped");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (m_is_running.exchange(true)) {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal request: can only handle 1 goal at the time");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "Accepting goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
Create3StraightCoverageNode::handle_cancel(
    const std::shared_ptr<GoalHandleCoverage> goal_handle)
{
    (void)goal_handle;

    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Create3StraightCoverageNode::handle_accepted(const std::shared_ptr<GoalHandleCoverage> goal_handle)
{
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&Create3StraightCoverageNode::execute, this, _1), goal_handle}.detach();
}

/**
 * @brief Execute the coverage action
 *
 * This method implements the main execution loop for the coverage action.
 * It runs in a separate thread to avoid blocking the main ROS executor.
 * The method creates a state machine, feeds it sensor data, and monitors
 * for cancellation requests or robot kidnapping. It also publishes feedback
 * when the active behavior changes.
 *
 * @param goal_handle Handle to the active goal being executed
 */
void Create3StraightCoverageNode::execute(const std::shared_ptr<GoalHandleCoverage> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    // Set up control loop rate according to parameter
    rclcpp::Rate loop_rate(m_rate_hz);
    const auto goal = goal_handle->get_goal();
    auto start_time = this->now();

    // Check if the robot has reflexes enabled or if we need to manually handle hazards
    // Reflexes are built-in safety behaviors that automatically respond to hazards
    bool robot_has_reflexes = this->reflexes_setup();

    // Create the state machine that implements the coverage algorithm
    auto state_machine = std::make_unique<StraightCoverageStateMachine>(
        *goal,
        this->get_clock(),
        this->get_logger(),
        m_dock_action_client,
        m_undock_action_client,
        m_cmd_vel_publisher,
        robot_has_reflexes);

    StraightCoverageStateMachine::CoverageOutput output;
    output.state = State::RUNNING;
    bool is_docked = false;
    bool is_kidnapped = false;
    
    // Main execution loop - continues until action completes or is cancelled
    do {
        // Prepare data structure with latest sensor readings
        Behavior::Data data;
        {
            // Use mutex to ensure thread-safe access to sensor data
            std::lock_guard<std::mutex> guard(m_mutex);

            data.hazards = m_last_hazards;
            data.dock = m_last_dock;
            data.pose = m_last_odom.pose.pose;
            data.opcodes = m_last_opcodes;

            // Clear IR opcode buffer periodically to avoid stale readings
            // This prevents the robot from reacting to IR signals that are no longer present
            if (this->now() - m_last_opcodes_cleared_time >= rclcpp::Duration(std::chrono::milliseconds(m_opcodes_buffer_ms))) {
                m_last_opcodes_cleared_time = this->now();
                m_last_opcodes.clear();
            }

            is_kidnapped = m_last_kidnap.is_kidnapped;
            is_docked = m_last_dock.is_docked;
        }

        // Check if there is a cancel request and handle it immediately if present
        if (goal_handle->is_canceling()) {
            m_is_running = false;
            state_machine->cancel();
            auto result = std::make_shared<CoverageAction::Result>();
            result->success = false;
            result->is_docked = is_docked;
            result->duration = this->now() - start_time;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled!");
            return;
        }

        // Check if the robot is kidnapped (physically picked up or moved)
        // Abort immediately to ensure safety and to prevent erroneous behavior
        if (is_kidnapped) {
            m_is_running = false;
            state_machine->cancel();
            auto result = std::make_shared<CoverageAction::Result>();
            result->success = false;
            result->is_docked = is_docked;
            result->duration = this->now() - start_time;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Aborting goal! Robot has been kidnapped!");
            return;
        }

        // Run one cycle of the state machine
        output = state_machine->execute(data);
        
        // Publish feedback when behavior changes to inform action clients
        if (m_last_behavior != output.current_behavior) {
            auto feedback = std::make_shared<CoverageAction::Feedback>();
            feedback->current_behavior = output.current_behavior;
            goal_handle->publish_feedback(feedback);
            m_last_behavior = output.current_behavior;
        }

        // Maintain control loop rate for consistent execution timing
        loop_rate.sleep();
        
    } while (output.state == State::RUNNING && rclcpp::ok());

    RCLCPP_INFO(this->get_logger(), "Coverage action terminated");

    // If ROS is still running, send final result
    if (rclcpp::ok()) {
        m_is_running = false;
        auto result = std::make_shared<CoverageAction::Result>();
        result->success = (output.state == State::SUCCESS);
        result->is_docked = is_docked;
        result->duration = this->now() - start_time;
        if (result->success) {
            goal_handle->succeed(result);
        } else {
            goal_handle->abort(result);
        }
    }
}

/**
 * @brief Set up and verify robot reflexes configuration
 *
 * Queries the robot's parameter server to determine if built-in safety reflexes
 * are properly configured and active. These reflexes include automatic responses
 * to bumps, cliff detections, and wheel drops.
 *
 * @return bool True if robot has reflexes enabled, false otherwise
 */
bool Create3StraightCoverageNode::reflexes_setup()
{
    bool robot_has_reflexes = true;

    // Parameter names for the different reflex settings
    const std::vector<std::string> param_names = {
        "reflexes.REFLEX_BUMP",
        "reflexes.REFLEX_CLIFF",
        "reflexes.REFLEX_WHEEL_DROP",
        "reflexes_enabled"
    };

    // Check if reflexes are active by querying the parameter service
    auto get_params_future = m_reflexes_param_client->get_parameters(param_names);
    auto parameters = get_params_future.get();
    bool all_enabled = true;
    bool all_disabled = true;
    
    // Analyze parameter results to determine if reflexes are enabled
    for (const rclcpp::Parameter& param : parameters) {
        all_enabled = all_enabled && param.as_bool();
        all_disabled = all_disabled && !param.as_bool();
    }

    // Log the reflex status based on parameter values
    if (all_enabled) {
        robot_has_reflexes = true;
        RCLCPP_INFO(this->get_logger(), "Reflexes are enabled on the robot!");
    } else if (all_disabled) {
        robot_has_reflexes = false;
        RCLCPP_WARN(this->get_logger(), "Reflexes are disabled on the robot!");
    } else {
        robot_has_reflexes = false;
        RCLCPP_ERROR(this->get_logger(), "Reflex setup is inconsistent!");
    }

    return robot_has_reflexes;
}

/**
 * @brief Check if the node is ready to start coverage
 *
 * This function performs a comprehensive check to verify that all required
 * communication channels with the robot are fully established before starting
 * a coverage operation. This includes checking that publishers, subscribers,
 * action clients, and parameter services are connected and ready.
 *
 * @return bool True if all required connections are established, false otherwise
 */
bool Create3StraightCoverageNode::ready_to_start()
{
    // Check that all subscriptions have discovered their publishers
    // This ensures we're receiving all necessary sensor data
    if (m_dock_subscription->get_publisher_count() == 0 ||
        m_hazards_subscription->get_publisher_count() == 0 ||
        m_ir_opcode_subscription->get_publisher_count() == 0 ||
        m_odom_subscription->get_publisher_count() == 0 ||
        m_kidnap_subscription->get_publisher_count() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "Some subscriptions haven't discovered their publishers yet");
        return false;
    }

    // Check that our velocity publisher has discovered subscribers
    // This ensures the robot will receive our movement commands
    if (m_cmd_vel_publisher->get_subscription_count() == 0) {
        RCLCPP_WARN(this->get_logger(), "Some publishers haven't discovered their subscriptions yet");
        return false;
    }

    // Check that parameter services are ready
    // This allows us to query or set robot parameters like reflexes
    if (!m_reflexes_param_client->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "Some parameters servers are not ready yet");
        return false;
    }

    // Check that action servers for docking/undocking are ready
    // This ensures we can send dock and undock commands to the robot
    if (!m_dock_action_client->action_server_is_ready() ||
        !m_undock_action_client->action_server_is_ready())
    {
        RCLCPP_WARN(this->get_logger(), "Some actions servers are not ready yet");
        return false;
    }

    // We must know if the robot is docked or not before starting the behavior
    // This is critical for selecting the initial behavior
    if (!m_dock_msgs_received) {
        RCLCPP_WARN(this->get_logger(), "Didn't receive a dock message yet");
        return false;
    }

    return true;
}

void Create3StraightCoverageNode::dock_callback(DockMsg::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_dock_msgs_received = true;
    m_last_dock = *msg;
}

void Create3StraightCoverageNode::hazards_callback(HazardMsg::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_last_hazards = *msg;
}

void Create3StraightCoverageNode::ir_opcode_callback(OpCodeMsg::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_last_opcodes.push_back(*msg);
}

void Create3StraightCoverageNode::kidnap_callback(KidnapMsg::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_last_kidnap = *msg;
}

void Create3StraightCoverageNode::odom_callback(OdometryMsg::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_last_odom = *msg;
}

} // namespace create3_straight_coverage
