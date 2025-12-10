/**
 * @file agent_params.hpp
 * @brief Agent parameters structure for heterogeneous fleet
 * @date 2024-12-10
 */

#pragma once

#include <cmath>
#include <string>

namespace libMultiRobotPlanning {

/**
 * @brief Structure to store per-agent kinematic parameters
 */
struct AgentParams {
  // Identification
  int agent_id;
  std::string agent_type;

  // Geometry (in meters)
  double length;        // Total length of robot
  double width;         // Width of robot
  double wheelbase;     // Distance between front and rear axles
  double LF;            // Distance from rear axle to front end
  double LB;            // Distance from rear axle to back end

  // Kinematic constraints
  double max_steering_angle;  // Maximum steering angle (radians)
  double min_turning_radius;  // Minimum turning radius (meters)
  double max_velocity;        // Maximum velocity (m/s)

  // Cost penalties
  double penalty_turning;    // Penalty for turning
  double penalty_reversing;  // Penalty for reversing

  // Priority for planning order (larger/stiffer robots first)
  double capability_score;

  // Derived parameters
  double deltat;  // Discretization angle
  double xyResolution;
  double yawResolution;

  // Motion primitive deltas
  double dx[6];
  double dy[6];
  double dyaw[6];

  /**
   * @brief Default constructor
   */
  AgentParams()
      : agent_id(0),
        agent_type("DEFAULT"),
        length(3.0),
        width(2.0),
        wheelbase(2.5),
        LF(2.0),
        LB(1.0),
        max_steering_angle(0.6),
        min_turning_radius(3.0),
        max_velocity(1.5),
        penalty_turning(1.5),
        penalty_reversing(2.0),
        capability_score(0.0),
        deltat(0.0),
        xyResolution(0.0),
        yawResolution(0.0) {
    computeDerivedParams();
  }

  /**
   * @brief Constructor with parameters
   */
  AgentParams(int id, const std::string& type, double len, double wid,
              double wb, double lf, double lb, double max_steer, double max_vel)
      : agent_id(id),
        agent_type(type),
        length(len),
        width(wid),
        wheelbase(wb),
        LF(lf),
        LB(lb),
        max_steering_angle(max_steer),
        max_velocity(max_vel),
        penalty_turning(1.5),
        penalty_reversing(2.0) {
    // Compute min turning radius from wheelbase and max steering angle
    min_turning_radius = wheelbase / std::tan(max_steering_angle);
    computeDerivedParams();
  }

  /**
   * @brief Compute derived parameters from base parameters
   */
  void computeDerivedParams() {
    // Discretization angle based on turning radius
    deltat = 6.75 * 6.0 / 180.0 * M_PI;  // ~0.706 radians

    // Resolution for indexing
    xyResolution = min_turning_radius * deltat;
    yawResolution = deltat;

    // Compute capability score for priority (larger/stiffer robots first)
    capability_score =
        0.3 * (length * width) + 0.7 * min_turning_radius;

    // Motion primitives: [straight, left, right, back, back-left, back-right]
    dx[0] = min_turning_radius * deltat;
    dx[1] = min_turning_radius * std::sin(deltat);
    dx[2] = min_turning_radius * std::sin(deltat);
    dx[3] = -min_turning_radius * deltat;
    dx[4] = -min_turning_radius * std::sin(deltat);
    dx[5] = -min_turning_radius * std::sin(deltat);

    dy[0] = 0;
    dy[1] = -min_turning_radius * (1 - std::cos(deltat));
    dy[2] = min_turning_radius * (1 - std::cos(deltat));
    dy[3] = 0;
    dy[4] = -min_turning_radius * (1 - std::cos(deltat));
    dy[5] = min_turning_radius * (1 - std::cos(deltat));

    dyaw[0] = 0;
    dyaw[1] = deltat;
    dyaw[2] = -deltat;
    dyaw[3] = 0;
    dyaw[4] = -deltat;
    dyaw[5] = deltat;
  }

  /**
   * @brief Get dx for a given action
   */
  double getDx(int action) const {
    if (action >= 0 && action < 6) return dx[action];
    return dx[0];  // Default to straight
  }

  /**
   * @brief Get dy for a given action
   */
  double getDy(int action) const {
    if (action >= 0 && action < 6) return dy[action];
    return 0.0;
  }

  /**
   * @brief Get dyaw for a given action
   */
  double getDyaw(int action) const {
    if (action >= 0 && action < 6) return dyaw[action];
    return 0.0;
  }
};

}  // namespace libMultiRobotPlanning
