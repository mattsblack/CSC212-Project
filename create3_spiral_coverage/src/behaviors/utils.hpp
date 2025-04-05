// Copyright 2021 iRobot Corporation. All Rights Reserved.

#pragma once

#include "geometry_msgs/msg/point.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "irobot_create_msgs/msg/ir_opcode.hpp"

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
double get_distance(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);

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
bool is_driving_towards_dock(const std::vector<irobot_create_msgs::msg::IrOpcode>& opcodes);

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
bool is_front_hazard_active(const irobot_create_msgs::msg::HazardDetectionVector& hazards);

} // namespace create3_spiral_coverage
