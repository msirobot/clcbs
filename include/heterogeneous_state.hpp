/**
 * @file heterogeneous_state.hpp
 * @brief Heterogeneous State with Agent-Specific Geometry
 * @date 2024-12-11
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <cmath>
#include <iostream>

#include "fleet_registry.hpp"

namespace libMultiRobotPlanning {

typedef boost::geometry::model::d2::point_xy<double> Point;
typedef boost::geometry::model::segment<Point> Segment;

/**
 * @brief Heterogeneous state with agent-specific geometry
 */
struct HeterogeneousState {
  double x;
  double y;
  double yaw;
  int time;
  size_t agent_id;
  
  HeterogeneousState() : x(0), y(0), yaw(0), time(0), agent_id(0) {}
  
  HeterogeneousState(double x, double y, double yaw, int time = 0, 
                     size_t agent_id = 0)
      : x(x), y(y), yaw(yaw), time(time), agent_id(agent_id) {}
  
  bool operator==(const HeterogeneousState& other) const {
    return time == other.time && 
           std::abs(x - other.x) < 1e-6 && 
           std::abs(y - other.y) < 1e-6 &&
           std::abs(yaw - other.yaw) < 1e-6 &&
           agent_id == other.agent_id;
  }
  
  /**
   * @brief Check collision with another agent using OBB
   * 
   * @param other Other agent's state
   * @param thisParams This agent's parameters
   * @param otherParams Other agent's parameters
   * @return true if collision detected
   */
  bool agentCollision(const HeterogeneousState& other,
                      const RobotParams& thisParams,
                      const RobotParams& otherParams) const {
    // Simple circle-based collision check with conservative radii
    double this_radius = std::sqrt(std::pow(thisParams.LF + thisParams.LB, 2) + 
                                   std::pow(thisParams.width_W, 2)) / 2.0;
    double other_radius = std::sqrt(std::pow(otherParams.LF + otherParams.LB, 2) + 
                                    std::pow(otherParams.width_W, 2)) / 2.0;
    
    double dist_sq = std::pow(this->x - other.x, 2) + 
                     std::pow(this->y - other.y, 2);
    
    return dist_sq < std::pow(this_radius + other_radius, 2);
  }
  
  /**
   * @brief Check collision with obstacle using robot dimensions
   * 
   * @param obstacle Obstacle location
   * @param params Agent's parameters
   * @param obsRadius Obstacle radius
   * @return true if collision detected
   */
  bool obsCollision(const Point& obstacle, const RobotParams& params,
                    double obsRadius = 1.0) const {
    // Rotate obstacle to robot frame
    boost::numeric::ublas::matrix<double> rot(2, 2);
    rot(0, 0) = std::cos(-this->yaw);
    rot(0, 1) = -std::sin(-this->yaw);
    rot(1, 0) = std::sin(-this->yaw);
    rot(1, 1) = std::cos(-this->yaw);
    
    boost::numeric::ublas::matrix<double> obs(1, 2);
    obs(0, 0) = obstacle.x() - this->x;
    obs(0, 1) = obstacle.y() - this->y;
    
    auto rotated_obs = boost::numeric::ublas::prod(obs, rot);
    
    // Check if obstacle intersects with robot bounding box
    if (rotated_obs(0, 0) > -params.LB - obsRadius &&
        rotated_obs(0, 0) < params.LF + obsRadius &&
        rotated_obs(0, 1) > -params.width_W / 2.0 - obsRadius &&
        rotated_obs(0, 1) < params.width_W / 2.0 + obsRadius) {
      return true;
    }
    
    return false;
  }
  
  friend std::ostream& operator<<(std::ostream& os, 
                                  const HeterogeneousState& s) {
    return os << "Agent" << s.agent_id << "(" << s.x << "," << s.y << ":" 
              << s.yaw << ")@" << s.time;
  }
};

}  // namespace libMultiRobotPlanning

namespace std {
template <>
struct hash<libMultiRobotPlanning::HeterogeneousState> {
  size_t operator()(const libMultiRobotPlanning::HeterogeneousState& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    boost::hash_combine(seed, s.yaw);
    boost::hash_combine(seed, s.agent_id);
    return seed;
  }
};
}  // namespace std
