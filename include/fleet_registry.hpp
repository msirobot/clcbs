/**
 * @file fleet_registry.hpp
 * @brief Fleet registry for managing heterogeneous robot fleet
 * @date 2024-12-10
 */

#pragma once

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "agent_params.hpp"

namespace libMultiRobotPlanning {

/**
 * @brief Central registry for managing heterogeneous fleet
 */
class FleetRegistry {
 public:
  FleetRegistry() = default;

  /**
   * @brief Register an agent in the fleet
   * @param params Agent parameters
   * @return true if registration successful
   */
  bool registerAgent(const AgentParams& params) {
    if (agents_.find(params.agent_id) != agents_.end()) {
      std::cerr << "Agent " << params.agent_id << " already registered\n";
      return false;
    }
    agents_[params.agent_id] = params;
    return true;
  }

  /**
   * @brief Get parameters for a specific agent
   * @param agent_id Agent identifier
   * @return Agent parameters
   */
  const AgentParams& getParams(int agent_id) const {
    auto it = agents_.find(agent_id);
    if (it != agents_.end()) {
      return it->second;
    }
    // Return default params if not found
    static AgentParams default_params;
    return default_params;
  }

  /**
   * @brief Check if agent is registered
   */
  bool hasAgent(int agent_id) const {
    return agents_.find(agent_id) != agents_.end();
  }

  /**
   * @brief Get all agents sorted by capability score (descending)
   * Higher capability score = larger/stiffer robots planned first
   */
  std::vector<int> getAgentsByPriority() const {
    std::vector<std::pair<double, int>> scored_agents;
    for (const auto& pair : agents_) {
      scored_agents.emplace_back(pair.second.capability_score, pair.first);
    }

    // Sort by score descending (larger robots first)
    std::sort(scored_agents.begin(), scored_agents.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });

    std::vector<int> result;
    for (const auto& pair : scored_agents) {
      result.push_back(pair.second);
    }
    return result;
  }

  /**
   * @brief Load fleet configuration from YAML file
   * @param filename Path to YAML configuration file
   * @return true if loading successful
   */
  bool loadFromYAML(const std::string& filename) {
    try {
      YAML::Node config = YAML::LoadFile(filename);

      if (!config["agents"]) {
        std::cerr << "No 'agents' section in fleet config\n";
        return false;
      }

      for (const auto& agent_node : config["agents"]) {
        AgentParams params;
        params.agent_id = agent_node["id"].as<int>();
        params.agent_type = agent_node["type"].as<std::string>();
        params.length = agent_node["length"].as<double>();
        params.width = agent_node["width"].as<double>();
        params.wheelbase = agent_node["wheelbase"].as<double>();
        params.LF = agent_node["LF"].as<double>();
        params.LB = agent_node["LB"].as<double>();
        params.max_steering_angle = agent_node["max_steering_angle"].as<double>();
        params.max_velocity = agent_node["max_velocity"].as<double>();

        // Optional parameters with defaults
        if (agent_node["penalty_turning"]) {
          params.penalty_turning = agent_node["penalty_turning"].as<double>();
        }
        if (agent_node["penalty_reversing"]) {
          params.penalty_reversing = agent_node["penalty_reversing"].as<double>();
        }

        // Compute derived parameters
        params.min_turning_radius =
            params.wheelbase / std::tan(params.max_steering_angle);
        params.computeDerivedParams();

        if (!registerAgent(params)) {
          std::cerr << "Failed to register agent " << params.agent_id << "\n";
          return false;
        }
      }

      return true;
    } catch (const std::exception& e) {
      std::cerr << "Error loading fleet config: " << e.what() << "\n";
      return false;
    }
  }

  /**
   * @brief Print summary of registered agents
   */
  void printSummary() const {
    std::cout << "=== Fleet Registry Summary ===\n";
    std::cout << "Total agents: " << agents_.size() << "\n\n";

    auto priority_list = getAgentsByPriority();
    for (int agent_id : priority_list) {
      const auto& params = agents_.at(agent_id);
      std::cout << "Agent " << agent_id << " (" << params.agent_type << "):\n";
      std::cout << "  Size: " << params.length << "m x " << params.width
                << "m\n";
      std::cout << "  Min turning radius: " << params.min_turning_radius
                << "m\n";
      std::cout << "  Max velocity: " << params.max_velocity << "m/s\n";
      std::cout << "  Capability score: " << params.capability_score << "\n\n";
    }
  }

  /**
   * @brief Get number of registered agents
   */
  size_t size() const { return agents_.size(); }

  /**
   * @brief Clear all registered agents
   */
  void clear() { agents_.clear(); }

 private:
  std::map<int, AgentParams> agents_;
};

}  // namespace libMultiRobotPlanning
