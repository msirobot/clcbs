/**
 * @file fleet_registry.hpp
 * @brief Fleet Registry System for heterogeneous robot management
 * @date 2024-12-11
 *
 * @copyright Copyright (c) 2024
 */

#pragma once

#include <map>
#include <string>
#include <vector>

namespace libMultiRobotPlanning {

// Default angular resolution constant: 6.75 * 6 degrees = 40.5 degrees
constexpr double DEFAULT_DELTA_T_RAD = 6.75 * 6.0 / 180.0 * M_PI;  // ~0.706 radians

/**
 * @brief Robot specification with kinematic parameters
 * 
 * Defines the physical and kinematic constraints of a robot type
 */
struct RobotSpec {
  std::string type;        // Robot type identifier (e.g., "Agile", "Standard", "Heavy")
  double length;           // Length from rear axle to front (LF) [m]
  double width;            // Width of the robot [m]
  double rear_length;      // Length from rear axle to back (LB) [m]
  double min_turning_radius;  // Minimum turning radius [m]
  double max_velocity;     // Maximum velocity [m/s]
  double delta_t;          // Angular resolution for motion primitives [rad]
  
  // Cost penalties
  double penalty_turning;   // Penalty for turning
  double penalty_reversing; // Penalty for reversing
  double penalty_cod;       // Penalty for change of direction
  
  /**
   * @brief Calculate capability score for priority batching
   * @param w1 Weight for size factor
   * @param w2 Weight for turning radius
   * @return Capability score (higher means less agile)
   */
  double capabilityScore(double w1 = 1.0, double w2 = 1.0) const {
    return w1 * (length * width) + w2 * min_turning_radius;
  }
  
  /**
   * @brief Get total length of the robot
   */
  double totalLength() const {
    return length + rear_length;
  }
};

/**
 * @brief Fleet Registry for managing heterogeneous robot specifications
 * 
 * Central storage and management system for all robot types in the fleet
 */
class FleetRegistry {
 public:
  FleetRegistry() {
    initializeDefaultSpecs();
  }
  
  /**
   * @brief Register a new robot specification
   * @param spec Robot specification to register
   */
  void registerRobotSpec(const RobotSpec& spec) {
    robot_specs_[spec.type] = spec;
  }
  
  /**
   * @brief Get robot specification by type
   * @param type Robot type identifier
   * @return Robot specification
   */
  const RobotSpec& getRobotSpec(const std::string& type) const {
    auto it = robot_specs_.find(type);
    if (it != robot_specs_.end()) {
      return it->second;
    }
    // Return default (Standard) if not found
    return robot_specs_.at("Standard");
  }
  
  /**
   * @brief Get all registered robot types
   * @return Vector of robot type names
   */
  std::vector<std::string> getRobotTypes() const {
    std::vector<std::string> types;
    for (const auto& pair : robot_specs_) {
      types.push_back(pair.first);
    }
    return types;
  }
  
  /**
   * @brief Check if a robot type is registered
   * @param type Robot type identifier
   * @return true if registered, false otherwise
   */
  bool hasRobotType(const std::string& type) const {
    return robot_specs_.find(type) != robot_specs_.end();
  }
  
 private:
  std::map<std::string, RobotSpec> robot_specs_;
  
  /**
   * @brief Initialize default robot specifications
   * 
   * Creates three default robot types:
   * - Agile: Small AGV with high agility
   * - Standard: Moderate transporter (default for backward compatibility)
   * - Heavy: Large forklift with low agility
   */
  void initializeDefaultSpecs() {
    // Type A: Agile - Small AGV with high agility
    RobotSpec agile;
    agile.type = "Agile";
    agile.length = 0.6;           // LF = 0.6m
    agile.width = 0.5;            // W = 0.5m
    agile.rear_length = 0.3;      // LB = 0.3m
    agile.min_turning_radius = 0.5;  // R_min = 0.5m
    agile.max_velocity = 2.0;     // v_max = 2.0 m/s
    agile.delta_t = DEFAULT_DELTA_T_RAD;
    agile.penalty_turning = 1.3;
    agile.penalty_reversing = 1.8;
    agile.penalty_cod = 1.8;
    robot_specs_["Agile"] = agile;
    
    // Type B: Standard - Moderate transporter (DEFAULT)
    RobotSpec standard;
    standard.type = "Standard";
    standard.length = 2.0;        // LF = 2.0m
    standard.width = 2.0;         // W = 2.0m
    standard.rear_length = 1.0;   // LB = 1.0m
    standard.min_turning_radius = 3.0;  // R_min = 3.0m (was 2.5m in spec, using 3.0 for original compatibility)
    standard.max_velocity = 1.2;  // v_max = 1.2 m/s
    standard.delta_t = DEFAULT_DELTA_T_RAD;
    standard.penalty_turning = 1.5;
    standard.penalty_reversing = 2.0;
    standard.penalty_cod = 2.0;
    robot_specs_["Standard"] = standard;
    
    // Type C: Heavy - Large forklift with low agility
    RobotSpec heavy;
    heavy.type = "Heavy";
    heavy.length = 3.0;           // LF = 3.0m
    heavy.width = 1.5;            // W = 1.5m
    heavy.rear_length = 1.5;      // LB = 1.5m
    heavy.min_turning_radius = 4.5;  // R_min = 4.5m
    heavy.max_velocity = 0.8;     // v_max = 0.8 m/s
    heavy.delta_t = DEFAULT_DELTA_T_RAD;
    heavy.penalty_turning = 1.8;
    heavy.penalty_reversing = 2.5;
    heavy.penalty_cod = 2.5;
    robot_specs_["Heavy"] = heavy;
  }
};

}  // namespace libMultiRobotPlanning
