/**
 * @file geometry_collision.hpp
 * @brief Oriented Bounding Box (OBB) collision detection for heterogeneous agents
 * @date 2024-12-10
 */

#pragma once

#include <cmath>
#include <vector>

#include "agent_params.hpp"

namespace libMultiRobotPlanning {

/**
 * @brief 2D point structure
 */
struct Point2D {
  double x;
  double y;

  Point2D() : x(0), y(0) {}
  Point2D(double x_, double y_) : x(x_), y(y_) {}

  Point2D operator+(const Point2D& other) const {
    return Point2D(x + other.x, y + other.y);
  }

  Point2D operator-(const Point2D& other) const {
    return Point2D(x - other.x, y - other.y);
  }

  Point2D operator*(double scalar) const {
    return Point2D(x * scalar, y * scalar);
  }

  double dot(const Point2D& other) const { return x * other.x + y * other.y; }

  double length() const { return std::sqrt(x * x + y * y); }
};

/**
 * @brief Oriented Bounding Box class
 */
class OrientedBoundingBox {
 public:
  Point2D center;
  Point2D axis[2];  // Local x and y axes
  double half_size[2];  // Half-width and half-height

  OrientedBoundingBox() {
    center = Point2D(0, 0);
    axis[0] = Point2D(1, 0);
    axis[1] = Point2D(0, 1);
    half_size[0] = 1.0;
    half_size[1] = 0.5;
  }

  /**
   * @brief Create OBB from agent state and parameters
   * @param x X position (rear axle center)
   * @param y Y position (rear axle center)
   * @param yaw Heading angle (radians)
   * @param params Agent parameters
   */
  static OrientedBoundingBox fromAgent(double x, double y, double yaw,
                                        const AgentParams& params) {
    OrientedBoundingBox obb;

    // Center is at geometric center of the robot
    // Rear axle is at the reference point, so center is offset forward
    double center_offset = (params.LF - params.LB) / 2.0;
    obb.center.x = x + center_offset * std::cos(-yaw);
    obb.center.y = y + center_offset * std::sin(-yaw);

    // Local axes (rotated by yaw)
    obb.axis[0].x = std::cos(-yaw);
    obb.axis[0].y = std::sin(-yaw);
    obb.axis[1].x = -std::sin(-yaw);
    obb.axis[1].y = std::cos(-yaw);

    // Half sizes
    obb.half_size[0] = params.length / 2.0;
    obb.half_size[1] = params.width / 2.0;

    return obb;
  }

  /**
   * @brief Get the 4 corners of the OBB
   */
  std::vector<Point2D> getCorners() const {
    std::vector<Point2D> corners;
    corners.reserve(4);

    Point2D x_offset = axis[0] * half_size[0];
    Point2D y_offset = axis[1] * half_size[1];

    corners.push_back(center + x_offset + y_offset);
    corners.push_back(center + x_offset - y_offset);
    corners.push_back(center - x_offset - y_offset);
    corners.push_back(center - x_offset + y_offset);

    return corners;
  }

  /**
   * @brief Project OBB onto an axis
   * @return min and max projection values
   */
  std::pair<double, double> projectOntoAxis(const Point2D& axis_norm) const {
    auto corners = getCorners();
    double min_proj = corners[0].dot(axis_norm);
    double max_proj = min_proj;

    for (size_t i = 1; i < corners.size(); ++i) {
      double proj = corners[i].dot(axis_norm);
      if (proj < min_proj) min_proj = proj;
      if (proj > max_proj) max_proj = proj;
    }

    return {min_proj, max_proj};
  }
};

/**
 * @brief Collision checker using Separating Axis Theorem (SAT)
 */
class CollisionChecker {
 public:
  /**
   * @brief Check collision between two oriented bounding boxes using SAT
   * @return true if collision detected
   */
  static bool checkOBBCollision(const OrientedBoundingBox& obb1,
                                 const OrientedBoundingBox& obb2) {
    // Test separation along all potential separating axes
    // For 2D OBBs, we test the 2 axes from each box (4 total)

    // Test axes from first OBB
    for (int i = 0; i < 2; ++i) {
      if (isSeparatingAxis(obb1, obb2, obb1.axis[i])) {
        return false;  // Separated, no collision
      }
    }

    // Test axes from second OBB
    for (int i = 0; i < 2; ++i) {
      if (isSeparatingAxis(obb1, obb2, obb2.axis[i])) {
        return false;  // Separated, no collision
      }
    }

    // No separating axis found, boxes overlap
    return true;
  }

  /**
   * @brief Check collision between two agents
   */
  static bool checkCollision(double x1, double y1, double yaw1,
                              const AgentParams& params1, double x2, double y2,
                              double yaw2, const AgentParams& params2) {
    auto obb1 = OrientedBoundingBox::fromAgent(x1, y1, yaw1, params1);
    auto obb2 = OrientedBoundingBox::fromAgent(x2, y2, yaw2, params2);
    return checkOBBCollision(obb1, obb2);
  }

  /**
   * @brief Check collision between agent and circular obstacle
   */
  static bool checkObstacleCollision(double x, double y, double yaw,
                                      const AgentParams& params, double obs_x,
                                      double obs_y, double obs_radius) {
    auto obb = OrientedBoundingBox::fromAgent(x, y, yaw, params);

    // Find closest point on OBB to obstacle center
    Point2D obs_center(obs_x, obs_y);
    Point2D to_obs = obs_center - obb.center;

    // Project onto OBB axes and clamp
    Point2D closest_point = obb.center;
    for (int i = 0; i < 2; ++i) {
      double dist = to_obs.dot(obb.axis[i]);
      if (dist > obb.half_size[i]) dist = obb.half_size[i];
      if (dist < -obb.half_size[i]) dist = -obb.half_size[i];
      closest_point = closest_point + obb.axis[i] * dist;
    }

    // Check if closest point is within obstacle radius
    Point2D diff = obs_center - closest_point;
    return diff.length() < obs_radius;
  }

 private:
  /**
   * @brief Test if an axis is a separating axis
   */
  static bool isSeparatingAxis(const OrientedBoundingBox& obb1,
                                 const OrientedBoundingBox& obb2,
                                 const Point2D& axis) {
    auto proj1 = obb1.projectOntoAxis(axis);
    auto proj2 = obb2.projectOntoAxis(axis);

    // Check if projections overlap
    return proj1.second < proj2.first || proj2.second < proj1.first;
  }
};

}  // namespace libMultiRobotPlanning
