// Copyright 2021 iRobot Corporation. All Rights Reserved.

#pragma once

#include <vector>

#include "create3_straight_coverage/state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "irobot_create_msgs/msg/dock_status.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "irobot_create_msgs/msg/ir_opcode.hpp"

namespace create3_straight_coverage {

/**
 * @brief Abstract base class for robot behaviors in the Create3 straight coverage system
 * 
 * This class defines the interface for all robot behaviors. Each behavior implements the
 * execute method to perform its specific task based on current sensor data. The behavior
 * system allows for modular implementation of different robot actions (like drive straight,
 * rotate, dock, etc.) that can be chained together by a state machine.
 */
class Behavior
{
public:
    /**
     * @brief Container structure for robot sensor and state data
     * 
     * Aggregates all necessary sensor and state information required by behaviors
     * to make decisions about robot actions.
     */
    struct Data {
        geometry_msgs::msg::Pose pose;              ///< Current robot pose (position and orientation)
        irobot_create_msgs::msg::HazardDetectionVector hazards;  ///< Detected hazards like obstacles and cliffs
        irobot_create_msgs::msg::DockStatus dock;   ///< Docking station visibility and connection status
        std::vector<irobot_create_msgs::msg::IrOpcode> opcodes;  ///< IR sensor readings including dock signals
    };

    /**
     * @brief Default constructor
     */
    Behavior() = default;

    /**
     * @brief Virtual destructor to allow proper destruction of derived classes
     */
    virtual ~Behavior() = default;

    /**
     * @brief Execute one iteration of the behavior
     * 
     * This is the main behavior implementation method that processes sensor data 
     * and executes one step of the behavior's action. It should be designed to be
     * called repeatedly in a control loop until it returns SUCCESS or FAILURE.
     * 
     * @param data Current robot sensor and state data
     * @return State Indicates the current state of execution (RUNNING, SUCCESS, or FAILURE)
     */
    virtual State execute(const Data & data) = 0;

    /**
     * @brief Get the behavior's unique identifier
     * 
     * Used by the state machine to identify which behavior is currently active,
     * particularly for providing feedback to action clients.
     * 
     * @return int32_t Behavior ID from the coverage action's feedback message constants
     */
    virtual int32_t get_id() const = 0;

    /**
     * @brief Clean up resources when behavior is cancelled
     * 
     * This method is called when a behavior is interrupted or cancelled. It should
     * perform any necessary cleanup actions like stopping robot motion, cancelling
     * actions, or releasing resources. Default implementation does nothing.
     */
    virtual void cleanup() {}
};

} // namespace create3_straight_coverage
