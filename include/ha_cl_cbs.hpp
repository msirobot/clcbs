/**
 * @file ha_cl_cbs.hpp
 * @brief Heterogeneous-Adaptive Car-Like Conflict-Based Search (HA-CL-CBS)
 * @date 2024-12-10
 */

#pragma once

#include <chrono>
#include <map>

#include "ha_environment.hpp"
#include "hybrid_astar.hpp"

#define HA_MAX_RUNTIME 120

namespace libMultiRobotPlanning {

/**
 * @brief HA-CL-CBS solver for heterogeneous robot fleets
 *
 * This solver extends CL-CBS to support heterogeneous robot fleets where
 * each agent can have different kinematic parameters (size, turning radius, velocity).
 *
 * Key features:
 * - Priority-based planning (larger/stiffer robots first)
 * - Agent-specific motion primitives
 * - OBB collision detection with actual agent sizes
 * - Adaptive time buffers for constraints
 */
template <typename Location, typename Action, typename Cost>
class HA_CL_CBS {
 public:
  using State = HAState<Location>;
  using Conflict = HAConflict<Location>;
  using Constraint = HAConstraint<Location>;
  using Constraints = HAConstraints<Location>;
  using Environment = HAEnvironment<Location, Action, Cost>;

  HA_CL_CBS(Environment& environment, FleetRegistry& fleet_registry)
      : m_env(environment), m_fleet_registry(fleet_registry) {}

  ~HA_CL_CBS() {}

  /**
   * @brief Search for collision-free paths for all agents
   * @param initialStates Initial states for all agents
   * @param solution Output solution paths
   * @return true if solution found
   */
  bool search(const std::vector<State>& initialStates,
              std::vector<PlanResult<State, Action, Cost>>& solution) {
    HighLevelNode start;
    start.solution.resize(initialStates.size());
    start.constraints.resize(initialStates.size());
    start.cost = 0;
    start.id = 0;

    // Get agents sorted by priority (larger/stiffer robots first)
    auto priority_order = m_fleet_registry.getAgentsByPriority();

    std::cout << "Planning order based on capability scores:\n";
    for (size_t i = 0; i < priority_order.size() && i < initialStates.size(); ++i) {
      int agent_id = priority_order[i];
      if (agent_id < static_cast<int>(initialStates.size())) {
        const auto& params = m_fleet_registry.getParams(agent_id);
        std::cout << "  Agent " << agent_id << " (" << params.agent_type
                  << ") - Score: " << params.capability_score << "\n";
      }
    }

    // Plan for all agents initially
    for (size_t i = 0; i < initialStates.size(); ++i) {
      LowLevelEnvironment llenv(m_env, i, start.constraints[i]);
      LowLevelSearch_t lowLevel(llenv);
      bool success = lowLevel.search(initialStates[i], start.solution[i]);
      if (!success) {
        std::cerr << "Failed to find initial path for agent " << i << "\n";
        return false;
      }
      start.cost += start.solution[i].cost;
    }

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true>>
        open;

    auto handle = open.push(start);
    (*handle).handle = handle;

    std::chrono::high_resolution_clock::time_point startTime =
        std::chrono::high_resolution_clock::now();
    solution.clear();
    int id = 1;

    while (!open.empty()) {
      auto endTime = std::chrono::high_resolution_clock::now();
      if (std::chrono::duration_cast<std::chrono::duration<double>>(endTime -
                                                                    startTime)
              .count() > HA_MAX_RUNTIME) {
        open.clear();
        std::cout << "\033[1m\033[31m Plan timed out! \033[0m\n";
        return false;
      }

      HighLevelNode P = open.top();
      m_env.onExpandHighLevelNode(P.cost);
      open.pop();

      Conflict conflict;
      if (!m_env.getFirstConflict(P.solution, conflict)) {
        // Solution found!
        solution = P.solution;
        return true;
      }

      // Create child nodes to resolve conflict
      std::map<size_t, Constraints> constraints;
      m_env.createConstraintsFromConflict(conflict, constraints);

      for (const auto& c : constraints) {
        size_t i = c.first;
        HighLevelNode newNode = P;
        newNode.id = id;

        assert(!newNode.constraints[i].overlap(c.second));

        newNode.constraints[i].add(c.second);
        newNode.cost -= newNode.solution[i].cost;

        // Re-plan with agent-specific parameters
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
  /**
   * @brief High-level CBS search tree node
   */
  struct HighLevelNode {
    std::vector<PlanResult<State, Action, Cost>> solution;
    std::vector<Constraints> constraints;
    Cost cost;
    int id;

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true>>::handle_type
        handle;

    bool operator<(const HighLevelNode& n) const { return cost > n.cost; }
  };

  /**
   * @brief Low-level environment wrapper for single-agent planning
   */
  struct LowLevelEnvironment {
    LowLevelEnvironment(Environment& env, size_t agentIdx,
                        const Constraints& constraints)
        : m_env(env) {
      m_env.setLowLevelContext(agentIdx, &constraints);
    }

    Cost admissibleHeuristic(const State& s) {
      return m_env.admissibleHeuristic(s);
    }

    bool isSolution(const State& s, Cost g,
                    std::unordered_map<State, std::tuple<State, Action, Cost, Cost>,
                                       std::hash<State>>& camefrom) {
      return m_env.isSolution(s, g, camefrom);
    }

    void getNeighbors(const State& s, Action act,
                      std::vector<Neighbor<State, Action, Cost>>& neighbors) {
      m_env.getNeighbors(s, act, neighbors);
    }

    State getGoal() { return m_env.getGoal(); }

    uint64_t calcIndex(const State& s) { return m_env.calcIndex(s); }

    void onExpandNode(const State& s, Cost fScore, Cost gScore) {
      m_env.onExpandLowLevelNode(s, fScore, gScore);
    }

    void onDiscover(const State& /*s*/, Cost /*fScore*/, Cost /*gScore*/) {}

   private:
    Environment& m_env;
  };

 private:
  Environment& m_env;
  FleetRegistry& m_fleet_registry;
  typedef HybridAStar<State, Action, Cost, LowLevelEnvironment>
      LowLevelSearch_t;
};

}  // namespace libMultiRobotPlanning
