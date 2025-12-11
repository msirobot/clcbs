/**
 * @file heterogeneous_environment.hpp
 * @brief Environment class for heterogeneous agents
 * @date 2024-12-11
 *
 * @copyright Copyright (c) 2024
 */

#pragma once

#include "environment.hpp"
#include "fleet_registry.hpp"

namespace libMultiRobotPlanning {

/**
 * @brief Heterogeneous robot kinematic parameters
 * 
 * Stores per-agent kinematic constraints for motion primitive generation
 */
struct HeterogeneousConstants {
  double r;                    // Minimum turning radius
  double deltat;               // Angular resolution
  double penaltyTurning;       // Turning penalty
  double penaltyReversing;     // Reversing penalty
  double penaltyCOD;           // Change of direction penalty
  double carWidth;             // Robot width
  double LF;                   // Front length
  double LB;                   // Rear length
  
  // Motion primitives
  std::vector<double> dx;
  std::vector<double> dy;
  std::vector<double> dyaw;
  
  /**
   * @brief Initialize from RobotSpec
   */
  void initFromSpec(const RobotSpec& spec) {
    r = spec.min_turning_radius;
    deltat = spec.delta_t;
    penaltyTurning = spec.penalty_turning;
    penaltyReversing = spec.penalty_reversing;
    penaltyCOD = spec.penalty_cod;
    carWidth = spec.width;
    LF = spec.length;
    LB = spec.rear_length;
    
    // Generate motion primitives based on turning radius
    dx = {r * deltat,
          r * sin(deltat),
          r * sin(deltat),
          -r * deltat,
          -r * sin(deltat),
          -r * sin(deltat)};
    dy = {0,
          -r * (1 - cos(deltat)),
          r * (1 - cos(deltat)),
          0,
          -r * (1 - cos(deltat)),
          r * (1 - cos(deltat))};
    dyaw = {0, deltat, -deltat, 0, -deltat, deltat};
  }
};

/**
 * @brief Environment for heterogeneous multi-agent path planning
 * 
 * Extends base Environment to support agents with different kinematic
 * constraints and physical dimensions
 */
template <typename Location, typename State, typename Action, typename Cost,
          typename Conflict, typename Constraint, typename Constraints>
class HeterogeneousEnvironment : public Environment<Location, State, Action, Cost,
                                                     Conflict, Constraint, Constraints> {
 public:
  using Base = Environment<Location, State, Action, Cost, Conflict, Constraint, Constraints>;
  
  HeterogeneousEnvironment(size_t maxx, size_t maxy,
                           std::unordered_set<Location> obstacles,
                           std::multimap<int, State> dynamic_obstacles,
                           std::vector<State> goals)
      : Base(maxx, dimy, std::move(obstacles), std::move(dynamic_obstacles),
             std::move(goals)),
        m_current_agent_params() {
    // Initialize with default (Standard) parameters
    RobotSpec default_spec;
    default_spec.min_turning_radius = 3.0;
    default_spec.delta_t = 6.75 * 6 / 180.0 * M_PI;
    default_spec.penalty_turning = 1.5;
    default_spec.penalty_reversing = 2.0;
    default_spec.penalty_cod = 2.0;
    default_spec.width = 2.0;
    default_spec.length = 2.0;
    default_spec.rear_length = 1.0;
    m_current_agent_params.initFromSpec(default_spec);
  }
  
  /**
   * @brief Set the robot type for the current agent
   * 
   * This must be called before low-level search to configure
   * the kinematic constraints for path planning
   */
  void setAgentType(size_t agentIdx, const RobotSpec& spec) {
    m_current_agent_idx = agentIdx;
    m_current_agent_params.initFromSpec(spec);
    m_agent_specs[agentIdx] = spec;
  }
  
  /**
   * @brief Get neighbors using agent-specific motion primitives
   * 
   * Overrides base getNeighbors to use per-agent kinematic constraints
   */
  void getNeighbors(const State& s, Action action,
                    std::vector<Neighbor<State, Action, double>>& neighbors) {
    neighbors.clear();
    const auto& params = m_current_agent_params;
    
    double g = params.dx[0];
    for (Action act = 0; act < 6; act++) {
      double xSucc, ySucc, yawSucc;
      g = params.dx[0];
      
      xSucc = s.x + params.dx[act] * cos(-s.yaw) -
              params.dy[act] * sin(-s.yaw);
      ySucc = s.y + params.dx[act] * sin(-s.yaw) +
              params.dy[act] * cos(-s.yaw);
      yawSucc = Constants::normalizeHeadingRad(s.yaw + params.dyaw[act]);
      
      if (act % 3 != 0) {  // penalize turning
        g = g * params.penaltyTurning;
      }
      if ((act < 3 && action >= 3) || (action < 3 && act >= 3)) {
        // penalize change of direction
        g = g * params.penaltyCOD;
      }
      if (act >= 3) {  // backwards
        g = g * params.penaltyReversing;
      }
      
      State tempState(xSucc, ySucc, yawSucc, s.time + 1,
                      params.LF, params.carWidth, params.LB);
      if (this->stateValid(tempState)) {
        neighbors.emplace_back(
            Neighbor<State, Action, double>(tempState, act, g));
      }
    }
    
    // wait action
    g = params.dx[0];
    State tempState(s.x, s.y, s.yaw, s.time + 1,
                    params.LF, params.carWidth, params.LB);
    if (this->stateValid(tempState)) {
      neighbors.emplace_back(Neighbor<State, Action, double>(tempState, 6, g));
    }
  }
  
  /**
   * @brief Admissible heuristic using agent-specific turning radius
   */
  double admissibleHeuristic(const State& s) {
    // Use Reeds-Shepp with agent-specific turning radius
    ompl::base::ReedsSheppStateSpace reedsSheppPath(m_current_agent_params.r);
    OmplState* rsStart = (OmplState*)reedsSheppPath.allocState();
    OmplState* rsEnd = (OmplState*)reedsSheppPath.allocState();
    rsStart->setXY(s.x, s.y);
    rsStart->setYaw(s.yaw);
    rsEnd->setXY(this->m_goals[this->m_agentIdx].x, this->m_goals[this->m_agentIdx].y);
    rsEnd->setYaw(this->m_goals[this->m_agentIdx].yaw);
    double reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
    
    // Euclidean distance
    double euclideanCost = sqrt(
        pow(this->m_goals[this->m_agentIdx].x - s.x, 2) +
        pow(this->m_goals[this->m_agentIdx].y - s.y, 2));
    
    // Holonomic heuristic
    double twoDoffset = sqrt(
        pow((s.x - static_cast<int>(s.x)) -
            (this->m_goals[this->m_agentIdx].x -
             static_cast<int>(this->m_goals[this->m_agentIdx].x)), 2) +
        pow((s.y - static_cast<int>(s.y)) -
            (this->m_goals[this->m_agentIdx].y -
             static_cast<int>(this->m_goals[this->m_agentIdx].y)), 2));
    double twoDCost =
        this->holonomic_cost_maps[this->m_agentIdx]
                           [static_cast<int>(s.x / Constants::mapResolution)]
                           [static_cast<int>(s.y / Constants::mapResolution)] -
        twoDoffset;
    
    return std::max({reedsSheppCost, euclideanCost, twoDCost});
  }
  
 private:
  HeterogeneousConstants m_current_agent_params;
  size_t m_current_agent_idx;
  std::map<size_t, RobotSpec> m_agent_specs;
};

}  // namespace libMultiRobotPlanning
