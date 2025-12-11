/**
 * @file ha_environment.hpp
 * @brief Heterogeneous-Aware Environment for HA-CL-CBS
 * @date 2024-12-10
 */

#pragma once

#include <ompl/base/State.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <boost/functional/hash.hpp>
#include <boost/geometry.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <unordered_map>
#include <unordered_set>

#include "agent_params.hpp"
#include "fleet_registry.hpp"
#include "geometry_collision.hpp"
#include "neighbor.hpp"
#include "planresult.hpp"

namespace libMultiRobotPlanning {

typedef ompl::base::SE2StateSpace::StateType OmplState;
typedef boost::geometry::model::d2::point_xy<double> Point;

// Forward declarations for heterogeneous types
template <typename Location>
struct HAState;

template <typename Location>
struct HAConflict;

template <typename Location>
struct HAConstraint;

template <typename Location>
struct HAConstraints;

/**
 * @brief State structure with time for heterogeneous agents
 */
template <typename Location>
struct HAState {
  HAState(double x = 0, double y = 0, double yaw = 0, int time = 0)
      : time(time), x(x), y(y), yaw(yaw) {}

  bool operator==(const HAState& s) const {
    return std::tie(time, x, y, yaw) == std::tie(s.time, s.x, s.y, s.yaw);
  }

  friend std::ostream& operator<<(std::ostream& os, const HAState& s) {
    return os << "(" << s.x << "," << s.y << ":" << s.yaw << ")@" << s.time;
  }

  int time;
  double x;
  double y;
  double yaw;
};

/**
 * @brief Conflict between two agents with their states
 */
template <typename Location>
struct HAConflict {
  int time;
  size_t agent1;
  size_t agent2;
  HAState<Location> s1;
  HAState<Location> s2;

  friend std::ostream& operator<<(std::ostream& os, const HAConflict& c) {
    os << c.time << ": Collision [" << c.agent1 << " vs " << c.agent2 << "]";
    return os;
  }
};

/**
 * @brief Constraint structure for heterogeneous agents
 */
template <typename Location>
struct HAConstraint {
  HAConstraint(int time, HAState<Location> s, size_t agentid)
      : time(time), s(s), agentid(agentid) {}
  HAConstraint() = default;

  int time;
  HAState<Location> s;
  size_t agentid;

  bool operator<(const HAConstraint& other) const {
    return std::tie(time, s.x, s.y, s.yaw, agentid) <
           std::tie(other.time, other.s.x, other.s.y, other.s.yaw,
                    other.agentid);
  }

  bool operator==(const HAConstraint& other) const {
    return std::tie(time, s.x, s.y, s.yaw, agentid) ==
           std::tie(other.time, other.s.x, other.s.y, other.s.yaw,
                    other.agentid);
  }
};

/**
 * @brief Constraints collection
 */
template <typename Location>
struct HAConstraints {
  std::unordered_set<HAConstraint<Location>> constraints;

  void add(const HAConstraints& other) {
    constraints.insert(other.constraints.begin(), other.constraints.end());
  }

  bool overlap(const HAConstraints& other) const {
    for (const auto& c : constraints) {
      if (other.constraints.count(c) > 0) return true;
    }
    return false;
  }
};

}  // namespace libMultiRobotPlanning

// Hash functions for HAState and HAConstraint
namespace std {
template <typename Location>
struct hash<libMultiRobotPlanning::HAState<Location>> {
  size_t operator()(const libMultiRobotPlanning::HAState<Location>& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    boost::hash_combine(seed, s.yaw);
    return seed;
  }
};

template <typename Location>
struct hash<libMultiRobotPlanning::HAConstraint<Location>> {
  size_t operator()(
      const libMultiRobotPlanning::HAConstraint<Location>& c) const {
    size_t seed = 0;
    boost::hash_combine(seed, c.time);
    boost::hash_combine(seed, c.s.x);
    boost::hash_combine(seed, c.s.y);
    boost::hash_combine(seed, c.s.yaw);
    boost::hash_combine(seed, c.agentid);
    return seed;
  }
};
}  // namespace std

namespace libMultiRobotPlanning {

static inline float normalizeHeadingRad(float t) {
  if (t < 0) {
    t = t - 2.f * M_PI * std::floor(t / (2.f * M_PI));
    return 2.f * M_PI + t;
  }
  return t - 2.f * M_PI * std::floor(t / (2.f * M_PI));
}

/**
 * @brief Heterogeneous-Aware Environment
 * This environment supports agents with different kinematic parameters
 */
template <typename Location, typename Action, typename Cost>
class HAEnvironment {
 public:
  using State = HAState<Location>;
  using Conflict = HAConflict<Location>;
  using Constraint = HAConstraint<Location>;
  using Constraints = HAConstraints<Location>;

  HAEnvironment(size_t maxx, size_t maxy,
                std::unordered_set<Location> obstacles,
                std::multimap<int, State> dynamic_obstacles,
                std::vector<State> goals, FleetRegistry& fleet_registry,
                double map_resolution = 2.0)
      : m_mapResolution(map_resolution),
        m_obstacles(std::move(obstacles)),
        m_dynamic_obstacles(std::move(dynamic_obstacles)),
        m_goals(std::move(goals)),
        m_fleet_registry(fleet_registry),
        m_agentIdx(0),
        m_constraints(nullptr),
        m_lastGoalConstraint(-1),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0) {
    m_dimx = static_cast<int>(maxx / m_mapResolution);
    m_dimy = static_cast<int>(maxy / m_mapResolution);
  }

  // High Level functions
  bool getFirstConflict(const std::vector<PlanResult<State, Action, Cost>>& solution,
                        Conflict& result) {
    int max_t = 0;
    for (const auto& sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }

    for (int t = 0; t < max_t; ++t) {
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1 = getState(i, solution, t);
        const AgentParams& params1 = m_fleet_registry.getParams(i);

        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2 = getState(j, solution, t);
          const AgentParams& params2 = m_fleet_registry.getParams(j);

          // Use OBB collision detection with actual agent sizes
          if (CollisionChecker::checkCollision(state1.x, state1.y, state1.yaw,
                                                params1, state2.x, state2.y,
                                                state2.yaw, params2)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.s1 = state1;
            result.s2 = state2;
            return true;
          }
        }
      }
    }
    return false;
  }

  void createConstraintsFromConflict(const Conflict& conflict,
                                      std::map<size_t, Constraints>& constraints) {
    // Adaptive time buffer can be based on agent size if needed
    // const AgentParams& params1 = m_fleet_registry.getParams(conflict.agent1);
    // const AgentParams& params2 = m_fleet_registry.getParams(conflict.agent2);

    Constraints c1;
    c1.constraints.emplace(
        Constraint(conflict.time, conflict.s2, conflict.agent2));
    constraints[conflict.agent1] = c1;

    Constraints c2;
    c2.constraints.emplace(
        Constraint(conflict.time, conflict.s1, conflict.agent1));
    constraints[conflict.agent2] = c2;
  }

  void onExpandHighLevelNode(Cost /*cost*/) {
    m_highLevelExpanded++;
    if (m_highLevelExpanded % 50 == 0)
      std::cout << "Expanded " << m_highLevelExpanded << " high-level nodes\n";
  }

  int highLevelExpanded() { return m_highLevelExpanded; }

  // Low Level functions
  void setLowLevelContext(size_t agentIdx, const Constraints* constraints) {
    assert(constraints);
    m_agentIdx = agentIdx;
    m_constraints = constraints;
    m_current_params = m_fleet_registry.getParams(agentIdx);
    m_lastGoalConstraint = -1;

    for (const auto& c : constraints->constraints) {
      if (std::abs(m_goals[m_agentIdx].x - c.s.x) < 1e-3 &&
          std::abs(m_goals[m_agentIdx].y - c.s.y) < 1e-3) {
        m_lastGoalConstraint = std::max(m_lastGoalConstraint, c.time);
      }
    }
  }

  Cost admissibleHeuristic(const State& s) {
    // Use agent-specific turning radius for Reeds-Shepp heuristic
    ompl::base::ReedsSheppStateSpace reedsSheppPath(
        m_current_params.min_turning_radius);
    OmplState* rsStart = (OmplState*)reedsSheppPath.allocState();
    OmplState* rsEnd = (OmplState*)reedsSheppPath.allocState();
    rsStart->setXY(s.x, s.y);
    rsStart->setYaw(s.yaw);
    rsEnd->setXY(m_goals[m_agentIdx].x, m_goals[m_agentIdx].y);
    rsEnd->setYaw(m_goals[m_agentIdx].yaw);
    double reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);

    double euclideanCost =
        std::sqrt(std::pow(m_goals[m_agentIdx].x - s.x, 2) +
                  std::pow(m_goals[m_agentIdx].y - s.y, 2));

    return std::max(reedsSheppCost, euclideanCost);
  }

  bool isSolution(
      const State& state, Cost gscore,
      std::unordered_map<State, std::tuple<State, Action, Cost, Cost>,
                         std::hash<State>>& _camefrom) {
    double goal_distance = std::sqrt(
        std::pow(state.x - m_goals[m_agentIdx].x, 2) +
        std::pow(state.y - m_goals[m_agentIdx].y, 2));

    // Goal region threshold based on agent size
    constexpr double GOAL_REGION_MULTIPLIER = 5.0;
    if (goal_distance > GOAL_REGION_MULTIPLIER * (m_current_params.LB + m_current_params.LF))
      return false;

    if (state.time <= m_lastGoalConstraint) return false;

    // Goal tolerance thresholds
    constexpr double POSITION_TOLERANCE_MULTIPLIER = 2.0;
    constexpr double HEADING_TOLERANCE_RAD = 0.5;  // ~28 degrees
    if (goal_distance < POSITION_TOLERANCE_MULTIPLIER * m_current_params.length &&
        std::abs(normalizeHeadingRad(state.yaw - m_goals[m_agentIdx].yaw)) < HEADING_TOLERANCE_RAD) {
      // Add goal state to path
      m_goals[m_agentIdx].time = state.time + 1;
      _camefrom.insert(std::make_pair(
          m_goals[m_agentIdx],
          std::make_tuple(state, 0, 0.0, gscore)));
      return true;
    }
    
    return false;
  }

  void getNeighbors(const State& s, Action action,
                    std::vector<Neighbor<State, Action, Cost>>& neighbors) {
    neighbors.clear();

    for (Action act = 0; act < 6; act++) {
      double g = m_current_params.getDx(0);
      double xSucc = s.x + m_current_params.getDx(act) * std::cos(-s.yaw) -
                     m_current_params.getDy(act) * std::sin(-s.yaw);
      double ySucc = s.y + m_current_params.getDx(act) * std::sin(-s.yaw) +
                     m_current_params.getDy(act) * std::cos(-s.yaw);
      double yawSucc =
          normalizeHeadingRad(s.yaw + m_current_params.getDyaw(act));

      if (act % 3 != 0) {
        g = g * m_current_params.penalty_turning;
      }
      if ((act < 3 && action >= 3) || (action < 3 && act >= 3)) {
        g = g * 2.0;  // Penalty for change of direction
      }
      if (act >= 3) {
        g = g * m_current_params.penalty_reversing;
      }

      State tempState(xSucc, ySucc, yawSucc, s.time + 1);
      if (stateValid(tempState)) {
        neighbors.emplace_back(Neighbor<State, Action, Cost>(tempState, act, g));
      }
    }

    // Wait action
    State tempState(s.x, s.y, s.yaw, s.time + 1);
    if (stateValid(tempState)) {
      neighbors.emplace_back(
          Neighbor<State, Action, Cost>(tempState, 6, m_current_params.getDx(0)));
    }
  }

  State getGoal() { return m_goals[m_agentIdx]; }

  uint64_t calcIndex(const State& s) {
    return (uint64_t)s.time * (2 * M_PI / m_current_params.deltat) *
               (m_dimx / m_current_params.xyResolution) *
               (m_dimy / m_current_params.xyResolution) +
           (uint64_t)(normalizeHeadingRad(s.yaw) /
                      m_current_params.yawResolution) *
               (m_dimx / m_current_params.xyResolution) *
               (m_dimy / m_current_params.xyResolution) +
           (uint64_t)(s.y / m_current_params.xyResolution) *
               (m_dimx / m_current_params.xyResolution) +
           (uint64_t)(s.x / m_current_params.xyResolution);
  }

  void onExpandLowLevelNode(const State& /*s*/, Cost /*fScore*/,
                            Cost /*gScore*/) {
    m_lowLevelExpanded++;
  }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }

 private:
  State getState(size_t agentIdx,
                 const std::vector<PlanResult<State, Action, Cost>>& solution,
                 size_t t) {
    assert(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size()) {
      return solution[agentIdx].states[t].first;
    }
    assert(!solution[agentIdx].states.empty());
    return solution[agentIdx].states.back().first;
  }

  bool stateValid(const State& s) {
    double x_ind = s.x / m_mapResolution;
    double y_ind = s.y / m_mapResolution;
    if (x_ind < 0 || x_ind >= m_dimx || y_ind < 0 || y_ind >= m_dimy)
      return false;

    // Check obstacle collisions with actual agent size
    for (const auto& obs : m_obstacles) {
      if (CollisionChecker::checkObstacleCollision(
              s.x, s.y, s.yaw, m_current_params, obs.x, obs.y, 1.0)) {
        return false;
      }
    }

    // Check dynamic obstacles
    auto it = m_dynamic_obstacles.equal_range(s.time);
    for (auto itr = it.first; itr != it.second; ++itr) {
      if (CollisionChecker::checkCollision(s.x, s.y, s.yaw, m_current_params,
                                            itr->second.x, itr->second.y,
                                            itr->second.yaw, m_current_params)) {
        return false;
      }
    }

    // Check constraints
    constexpr int DEFAULT_CONSTRAINT_TIME_BUFFER = 2;  // timesteps
    for (const auto& c : m_constraints->constraints) {
      int time_buffer = DEFAULT_CONSTRAINT_TIME_BUFFER;
      if (s.time >= c.time && s.time <= c.time + time_buffer) {
        const AgentParams& other_params = m_fleet_registry.getParams(c.agentid);
        if (CollisionChecker::checkCollision(s.x, s.y, s.yaw, m_current_params,
                                              c.s.x, c.s.y, c.s.yaw,
                                              other_params)) {
          return false;
        }
      }
    }

    return true;
  }

 private:
  int m_dimx;
  int m_dimy;
  double m_mapResolution;
  std::unordered_set<Location> m_obstacles;
  std::multimap<int, State> m_dynamic_obstacles;
  std::vector<State> m_goals;
  FleetRegistry& m_fleet_registry;
  size_t m_agentIdx;
  const Constraints* m_constraints;
  AgentParams m_current_params;
  int m_lastGoalConstraint;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
};

}  // namespace libMultiRobotPlanning
