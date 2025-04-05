// Copyright 2021 iRobot Corporation. All Rights Reserved.

#include <math.h>

#include "utils.hpp"

#include <iostream>
namespace create3_spiral_coverage {

/**
 * @brief Calculates the Euclidean distance between two 3D points
 * 
 * This function computes the straight-line distance between two points in a 
 * Cartesian coordinate system using the Pythagorean formula. Only x and y 
 * coordinates are used for distance calculation (2D plane).
 *
 * @param p1 First point with x,y,z coordinates
 * @param p2 Second point with x,y,z coordinates
 * @return double The Euclidean distance between p1 and p2 in the same units as the input points
 */
double get_distance(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
    // Calculate differences in x and y coordinates (ignoring z)
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    
    // Return Euclidean distance √(dx² + dy²)
    return sqrt(dx*dx + dy*dy);
}

/**
 * @brief Determines if the robot is currently facing and close to its docking station
 * 
 * Analyzes IR opcodes from the robot's sensors to determine if it is both facing and
 * within proximity of the dock. The robot is considered to be driving towards the dock
 * when both directional (front-facing) and omnidirectional IR sensors detect dock signals
 * simultaneously. This indicates the robot is properly aligned for docking.
 *
 * @param opcodes Vector of IR opcode messages received from the robot's sensors
 * @return bool True if both directional and omnidirectional sensors detect the dock,
 *         false otherwise
 */
bool is_driving_towards_dock(const std::vector<irobot_create_msgs::msg::IrOpcode>& opcodes)
{
    using irobot_create_msgs::msg::IrOpcode;

    // We consider the robot to be driving towards the dock if both IR receivers are detecting it.
    // The directional sensor will tell us that we are pointing towards it.
    // The omni sensors will tell us that we are close to it.

    bool dir_dock_detection = false;  // Flag for directional sensor detection
    bool omni_dock_detection = false; // Flag for omnidirectional sensor detection

    // Iterate through all received IR opcodes
    for (const IrOpcode& msg : opcodes) {
        // Check if the front directional sensor detects anything other than a virtual wall
        if (msg.sensor == IrOpcode::SENSOR_DIRECTIONAL_FRONT && msg.opcode != IrOpcode::CODE_IR_VIRTUAL_WALL) {
            dir_dock_detection = true;
            continue;
        }
        // Check if the omnidirectional sensor detects anything other than a virtual wall
        if (msg.sensor == IrOpcode::SENSOR_OMNI && msg.opcode != IrOpcode::CODE_IR_VIRTUAL_WALL) {
            omni_dock_detection = true;
            continue;
        }
    }

    // Return true only if both sensors detected dock signals
    return dir_dock_detection && omni_dock_detection;    
}

/**
 * @brief Checks if any forward-facing hazard sensors are currently detecting obstacles
 * 
 * Examines the hazard detection vector to determine if the robot is detecting
 * any obstacles that would prevent forward motion. This function specifically excludes
 * BACKUP_LIMIT hazards since those only affect backward motion and don't impede
 * forward progress.
 *
 * @param hazards Vector of hazard detections from the robot's sensors
 * @return bool True if any forward hazard is detected, false if no hazards or only
 *         backup limit hazards are present
 */
bool is_front_hazard_active(const irobot_create_msgs::msg::HazardDetectionVector& hazards)
{
    using irobot_create_msgs::msg::HazardDetection;

    // Iterate through all hazard detections
    for (const HazardDetection& detection : hazards.detections) {
        // Consider any hazard type except BACKUP_LIMIT as an active front hazard
        // BACKUP_LIMIT only applies to backward motion, so we ignore it
        if (detection.type != HazardDetection::BACKUP_LIMIT) {
            return true;
        }
    }

    // No front hazards detected
    return false;
}

} // namespace create3_spiral_coverage
