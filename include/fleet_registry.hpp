/**
 * @file fleet_registry.hpp
 * @brief Fleet Registry for Heterogeneous Robot Parameters
 * @date 2024-12-11
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <map>
#include <string>
#include <stdexcept>

namespace Constants {
// Capability score weights
static const double CAPABILITY_SIZE_WEIGHT = 0.6;
static const double CAPABILITY_RMIN_WEIGHT = 0.4;
}  // namespace Constants

namespace libMultiRobotPlanning {

/**
 * @brief Robot kinematic and geometric parameters
 */
struct RobotParams {
  double wheelbase_L;      ///< Wheelbase length
  double width_W;          ///< Robot width
  double length;           ///< Total robot length (LF + LB)
  double LF;               ///< Distance from rear axle to front
  double LB;               ///< Distance from rear axle to back
  double R_min;            ///< Minimum turning radius
  double v_max;            ///< Maximum velocity
  double delta_t;          ///< Steering angle resolution
  double penaltyTurning;   ///< Penalty for turning motions
  double penaltyReversing; ///< Penalty for reversing motions
  std::string robot_type;  ///< Robot type identifier (AGILE, STANDARD, HEAVY)
  
  RobotParams()
      : wheelbase_L(1.2), width_W(1.0), length(1.5), LF(1.0), LB(0.5),
        R_min(2.5), v_max(1.5), delta_t(0.12), penaltyTurning(1.5),
        penaltyReversing(2.0), robot_type("STANDARD") {}
  
  RobotParams(double wheelbase, double width, double lf, double lb,
              double r_min, double vmax, double deltat, double penaltyTurn,
              double penaltyReverse, const std::string& type)
      : wheelbase_L(wheelbase), width_W(width), length(lf + lb), LF(lf),
        LB(lb), R_min(r_min), v_max(vmax), delta_t(deltat),
        penaltyTurning(penaltyTurn), penaltyReversing(penaltyReverse),
        robot_type(type) {}
};

/**
 * @brief Fleet Registry manages robot parameters for heterogeneous agents
 */
class FleetRegistry {
 public:
  FleetRegistry() {}
  
  /**
   * @brief Register an agent with its parameters
   * @param agent_id Agent identifier
   * @param params Robot parameters
   */
  void RegisterAgent(size_t agent_id, const RobotParams& params) {
    agents_[agent_id] = params;
  }
  
  /**
   * @brief Get parameters for a specific agent
   * @param agent_id Agent identifier
   * @return Robot parameters
   * @throws std::out_of_range if agent not registered
   */
  const RobotParams& GetParams(size_t agent_id) const {
    auto it = agents_.find(agent_id);
    if (it == agents_.end()) {
      throw std::out_of_range("Agent not registered in FleetRegistry");
    }
    return it->second;
  }
  
  /**
   * @brief Check if agent is registered
   * @param agent_id Agent identifier
   * @return true if registered, false otherwise
   */
  bool HasAgent(size_t agent_id) const {
    return agents_.find(agent_id) != agents_.end();
  }
  
  /**
   * @brief Calculate capability score for an agent
   * 
   * Capability score determines planning priority:
   * Higher score = larger/less maneuverable = planned first
   * 
   * @param agent_id Agent identifier
   * @return Capability score (higher = less capable/more constrained)
   */
  double GetCapabilityScore(size_t agent_id) const {
    const auto& params = GetParams(agent_id);
    // Weighted combination: size (60%) + turning constraint (40%)
    return Constants::CAPABILITY_SIZE_WEIGHT * (params.LF + params.LB) * params.width_W + 
           Constants::CAPABILITY_RMIN_WEIGHT * params.R_min;
  }
  
  /**
   * @brief Get number of registered agents
   * @return Number of agents
   */
  size_t GetNumAgents() const {
    return agents_.size();
  }
  
  /**
   * @brief Clear all registered agents
   */
  void Clear() {
    agents_.clear();
  }
  
 private:
  std::map<size_t, RobotParams> agents_;
};

}  // namespace libMultiRobotPlanning
