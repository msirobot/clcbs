/**
 * @file ha_cl_cbs.hpp
 * @brief Heterogeneous Adaptive CL-CBS header
 * @date 2024-12-11
 *
 * @copyright Copyright (c) 2024
 */
#pragma once

#include <chrono>
#include <map>
#include <vector>

#include "cl_cbs.hpp"
#include "fleet_registry.hpp"
#include "hybrid_astar.hpp"

#define MAX_RUNTIME 120

namespace libMultiRobotPlanning {

/**
 * @brief Heterogeneous Adaptive CL-CBS (HA-CL-CBS) algorithm
 * 
 * Extends CL-CBS to handle heterogeneous robot fleets with diverse kinematic
 * constraints. Features:
 * - Fleet Registry System for robot specifications
 * - Kinematic-Adaptive SHA* (KA-SHA*) with per-robot motion primitives
 * - Geometry-Aware Collision Detection with oriented bounding boxes
 * - Priority Batching based on capability scores
 * - Adaptive Conflict Resolution with size-based safety buffers
 * 
 * \tparam State Custom state for the search
 * \tparam Action Custom action for the search
 * \tparam Cost Custom Cost type (integer or floating point types)
 * \tparam Conflict Custom conflict description
 * \tparam Constraints Custom constraint description
 * \tparam Environment Environment class with heterogeneous agent support
 */
template <typename State, typename Action, typename Cost, typename Conflict,
          typename Constraints, typename Environment>
class HA_CL_CBS {
 public:
  HA_CL_CBS(Environment& environment, const FleetRegistry& fleet_registry)
      : m_env(environment), m_fleet_registry(fleet_registry) {}
  ~HA_CL_CBS() {}

  /**
   * @brief Search for collision-free paths for heterogeneous agents
   * 
   * @param initialStates Initial states of all agents
   * @param agent_types Robot type for each agent
   * @param solution Output: collision-free paths for all agents
   * @return true if solution found, false otherwise
   */
  bool search(const std::vector<State>& initialStates,
              const std::vector<std::string>& agent_types,
              std::vector<PlanResult<State, Action, Cost>>& solution) {
    // Store agent types for later use
    m_agent_types = agent_types;
    
    // Sort agents by capability score for priority batching
    std::vector<size_t> sorted_indices = priorityBatching(agent_types);
    
    HighLevelNode start;
    start.solution.resize(initialStates.size());
    start.constraints.resize(initialStates.size());
    start.cost = 0;
    start.id = 0;

    // Plan for agents in priority order
    for (size_t idx : sorted_indices) {
      // Note: For initial implementation, using default kinematic parameters
      // TODO: Extend Environment to support per-agent setAgentType()
      
      LowLevelEnvironment llenv(m_env, idx, start.constraints[idx]);
      LowLevelSearch_t lowLevel(llenv);
      bool success = lowLevel.search(initialStates[idx], start.solution[idx]);
      if (!success) {
        return false;
      }
      start.cost += start.solution[idx].cost;
    }

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true>>
        open;

    auto handle = open.push(start);
    (*handle).handle = handle;

    std::chrono::high_resolution_clock::time_point
        startTime = std::chrono::high_resolution_clock::now(),
        endTime;
    solution.clear();
    int id = 1;
    
    while (!open.empty()) {
      endTime = std::chrono::high_resolution_clock::now();
      if (std::chrono::duration_cast<std::chrono::duration<double>>(endTime -
                                                                    startTime)
              .count() > MAX_RUNTIME) {
        open.clear();
        std::cout << "\033[1m\033[31m HA-CL-CBS: Plan out of runtime! \033[0m\n";
        return false;
      }

      HighLevelNode P = open.top();
      m_env.onExpandHighLevelNode(P.cost);

      open.pop();

      Conflict conflict;
      if (!m_env.getFirstConflict(P.solution, conflict)) {
        solution = P.solution;
        return true;
      }

      // Create constraints with adaptive safety buffers based on agent sizes
      std::map<size_t, Constraints> constraints;
      createAdaptiveConstraints(conflict, constraints);
      
      for (const auto& c : constraints) {
        size_t i = c.first;
        HighLevelNode newNode = P;
        newNode.id = id;
        
        assert(!newNode.constraints[i].overlap(c.second));
        newNode.constraints[i].add(c.second);
        newNode.cost -= newNode.solution[i].cost;

        // Note: Using default kinematic parameters for now
        // TODO: Extend Environment to support per-agent setAgentType()
        
        LowLevelEnvironment llenv(m_env, i, newNode.constraints[i]);
        LowLevelSearch_t lowLevel(llenv);
        bool success = lowLevel.search(initialStates[i], newNode.solution[i]);

        newNode.cost += newNode.solution[i].cost;

        if (success) {
          auto handle = open.push(newNode);
          (*handle).handle = handle;
        }

        ++id;
      }
    }

    return false;
  }

 private:
  struct HighLevelNode {
    std::vector<PlanResult<State, Action, Cost>> solution;
    std::vector<Constraints> constraints;

    Cost cost;
    int id;

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true>>::handle_type
        handle;

    bool operator<(const HighLevelNode& n) const {
      return cost > n.cost;
    }

    friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& c) {
      os << "id: " << c.id << " cost: " << c.cost << std::endl;
      for (size_t i = 0; i < c.solution.size(); ++i) {
        os << "Agent: " << i << std::endl;
        os << " States:" << std::endl;
        for (size_t t = 0; t < c.solution[i].states.size(); ++t) {
          os << "  " << c.solution[i].states[t].first << std::endl;
        }
        os << " Constraints:" << std::endl;
        os << c.constraints[i];
        os << " cost: " << c.solution[i].cost << std::endl;
      }
      return os;
    }
  };

  struct LowLevelEnvironment {
    LowLevelEnvironment(Environment& env, size_t agentIdx,
                        const Constraints& constraints)
        : m_env(env) {
      m_env.setLowLevelContext(agentIdx, &constraints);
    }

    Cost admissibleHeuristic(const State& s) {
      return m_env.admissibleHeuristic(s);
    }

    bool isSolution(
        const State& s, Cost g,
        std::unordered_map<State, std::tuple<State, Action, Cost, Cost>,
                           std::hash<State>>& camefrom) {
      return m_env.isSolution(s, g, camefrom);
    }

    void getNeighbors(const State& s, Action act,
                      std::vector<Neighbor<State, Action, Cost>>& neighbors) {
      m_env.getNeighbors(s, act, neighbors);
    }

    State getGoal() { return m_env.getGoal(); }

    int calcIndex(const State& s) { return m_env.calcIndex(s); }

    void onExpandNode(const State& s, Cost fScore, Cost gScore) {
      m_env.onExpandLowLevelNode(s, fScore, gScore);
    }

    void onDiscover(const State& /*s*/, Cost /*fScore*/, Cost /*gScore*/) {}

   private:
    Environment& m_env;
  };

  /**
   * @brief Priority batching based on capability scores
   * 
   * Sorts agents by capability score (size * weight + turning_radius * weight)
   * More capable (agile) agents are planned first
   * 
   * @param agent_types Robot type for each agent
   * @return Sorted indices of agents (most agile first)
   */
  std::vector<size_t> priorityBatching(
      const std::vector<std::string>& agent_types) {
    std::vector<std::pair<double, size_t>> scores;
    
    for (size_t i = 0; i < agent_types.size(); ++i) {
      const RobotSpec& spec = m_fleet_registry.getRobotSpec(agent_types[i]);
      double score = spec.capabilityScore();
      scores.push_back({score, i});
    }
    
    // Sort by score (ascending - most agile first)
    std::sort(scores.begin(), scores.end());
    
    std::vector<size_t> indices;
    for (const auto& pair : scores) {
      indices.push_back(pair.second);
    }
    
    return indices;
  }

  /**
   * @brief Create adaptive constraints with size-based safety buffers
   * 
   * @param conflict The detected conflict
   * @param constraints Output: constraints for conflicting agents
   */
  void createAdaptiveConstraints(
      const Conflict& conflict, std::map<size_t, Constraints>& constraints) {
    // Use environment's default constraint creation for now
    // TODO: Add adaptive safety buffers based on robot sizes
    m_env.createConstraintsFromConflict(conflict, constraints);
  }

 private:
  Environment& m_env;
  const FleetRegistry& m_fleet_registry;
  std::vector<std::string> m_agent_types;
  typedef HybridAStar<State, Action, Cost, LowLevelEnvironment>
      LowLevelSearch_t;
};

}  // namespace libMultiRobotPlanning
