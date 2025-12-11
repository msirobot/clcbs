/**
 * @file ha_cl_cbs.cpp
 * @brief Heterogeneous Adaptive CL-CBS Implementation
 * @date 2024-12-11
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "cl_cbs.hpp"

#include <sys/stat.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <boost/functional/hash.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>
#include <algorithm>

#include "environment.hpp"
#include "fleet_registry.hpp"
#include "heterogeneous_state.hpp"
#include "timer.hpp"

using libMultiRobotPlanning::CL_CBS;
using libMultiRobotPlanning::FleetRegistry;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::RobotParams;
using namespace libMultiRobotPlanning;

struct Location {
  Location(double x, double y) : x(x), y(y) {}
  double x;
  double y;

  bool operator<(const Location& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const Location& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const Location& c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
};

namespace std {
template <>
struct hash<Location> {
  size_t operator()(const Location& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

// State for heterogeneous robots
struct State {
  State(double x, double y, double yaw, int time = 0, size_t agent_id = 0)
      : time(time), x(x), y(y), yaw(yaw), agent_id(agent_id) {
    rot.resize(2, 2);
    rot(0, 0) = cos(-this->yaw);
    rot(0, 1) = -sin(-this->yaw);
    rot(1, 0) = sin(-this->yaw);
    rot(1, 1) = cos(-this->yaw);
  }

  State() = default;

  bool operator==(const State& s) const {
    return std::tie(time, x, y, yaw, agent_id) == 
           std::tie(s.time, s.x, s.y, s.yaw, s.agent_id);
  }

  // Agent collision using fleet registry for dimensions
  bool agentCollision(const State& other, const FleetRegistry* registry = nullptr) const {
    if (registry && registry->HasAgent(agent_id) && registry->HasAgent(other.agent_id)) {
      const auto& thisParams = registry->GetParams(agent_id);
      const auto& otherParams = registry->GetParams(other.agent_id);
      
      double this_radius = sqrt(pow(thisParams.LF + thisParams.LB, 2) + 
                               pow(thisParams.width_W, 2)) / 2.0;
      double other_radius = sqrt(pow(otherParams.LF + otherParams.LB, 2) + 
                                pow(otherParams.width_W, 2)) / 2.0;
      
      return pow(this->x - other.x, 2) + pow(this->y - other.y, 2) < 
             pow(this_radius + other_radius, 2);
    }
    // Fallback to default collision check using diagonal radius
    double default_radius = sqrt(pow(Constants::LF + Constants::LB, 2) + 
                                pow(Constants::carWidth, 2)) / 2.0;
    return pow(this->x - other.x, 2) + pow(this->y - other.y, 2) < 
           pow(2 * default_radius, 2);
  }

  bool obsCollision(const Location& obstacle, const FleetRegistry* registry = nullptr) const {
    boost::numeric::ublas::matrix<double> obs(1, 2);
    obs(0, 0) = obstacle.x - this->x;
    obs(0, 1) = obstacle.y - this->y;

    auto rotated_obs = boost::numeric::ublas::prod(obs, rot);
    
    double LF = Constants::LF;
    double LB = Constants::LB;
    double width = Constants::carWidth;
    double obsRadius = Constants::obsRadius;
    
    if (registry && registry->HasAgent(agent_id)) {
      const auto& params = registry->GetParams(agent_id);
      LF = params.LF;
      LB = params.LB;
      width = params.width_W;
    }
    
    if (rotated_obs(0, 0) > -LB - obsRadius &&
        rotated_obs(0, 0) < LF + obsRadius &&
        rotated_obs(0, 1) > -width / 2.0 - obsRadius &&
        rotated_obs(0, 1) < width / 2.0 + obsRadius)
      return true;
    return false;
  }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << "Agent" << s.agent_id << "(" << s.x << "," << s.y << ":" 
              << s.yaw << ")@" << s.time;
  }

  int time;
  double x;
  double y;
  double yaw;
  size_t agent_id;

 private:
  boost::numeric::ublas::matrix<double> rot;
};

namespace std {
template <>
struct hash<State> {
  size_t operator()(const State& s) const {
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

using Action = int;

struct Conflict {
  int time;
  size_t agent1;
  size_t agent2;
  State s1;
  State s2;

  friend std::ostream& operator<<(std::ostream& os, const Conflict& c) {
    os << c.time << ": Collision [ " << c.agent1 << c.s1 << " , " << c.agent2
       << c.s2 << " ]";
    return os;
  }
};

struct Constraint {
  Constraint(int time, State s, size_t agentid)
      : time(time), s(s), agentid(agentid) {}
  Constraint() = default;
  int time;
  State s;
  size_t agentid;

  bool operator<(const Constraint& other) const {
    return std::tie(time, s.x, s.y, s.yaw, agentid) <
           std::tie(other.time, other.s.x, other.s.y, other.s.yaw,
                    other.agentid);
  }

  bool operator==(const Constraint& other) const {
    return std::tie(time, s.x, s.y, s.yaw, agentid) ==
           std::tie(other.time, other.s.x, other.s.y, other.s.yaw,
                    other.agentid);
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraint& c) {
    return os << "Constraint[" << c.time << "," << c.s << "from " << c.agentid
              << "]";
  }

  bool satisfyConstraint(const State& state, const FleetRegistry* registry = nullptr) const {
    if (state.time < this->time ||
        state.time > this->time + Constants::constraintWaitTime)
      return true;
    return !this->s.agentCollision(state, registry);
  }
};

namespace std {
template <>
struct hash<Constraint> {
  size_t operator()(const Constraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.s.x);
    boost::hash_combine(seed, s.s.y);
    boost::hash_combine(seed, s.s.yaw);
    boost::hash_combine(seed, s.agentid);
    return seed;
  }
};
}  // namespace std

struct Constraints {
  std::unordered_set<Constraint> constraints;

  void add(const Constraints& other) {
    constraints.insert(other.constraints.begin(), other.constraints.end());
  }

  bool overlap(const Constraints& other) {
    for (const auto& c : constraints) {
      if (other.constraints.count(c) > 0) return true;
    }
    return false;
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraints& cs) {
    for (const auto& c : cs.constraints) {
      os << c << std::endl;
    }
    return os;
  }
};

// Load robot type configurations
FleetRegistry loadRobotTypes(const std::string& config_file) {
  FleetRegistry registry;
  
  YAML::Node config;
  try {
    config = YAML::LoadFile(config_file);
  } catch (std::exception& e) {
    std::cerr << "\033[1m\033[33mWARNING: Failed to load config file: "
              << config_file << "\033[0m. Cannot proceed without configuration.\n";
    return registry;
  }
  
  if (!config["robot_types"]) {
    std::cerr << "\033[1m\033[33mWARNING: No robot_types found in config\033[0m\n";
    return registry;
  }
  
  // Note: We'll register agents later when we know their IDs from the map file
  return registry;
}

// Read agent configuration from YAML and register agents
void registerAgentsFromYAML(FleetRegistry& registry, 
                             const YAML::Node& map_config,
                             const YAML::Node& robot_config) {
  if (!map_config["agents"]) {
    std::cerr << "\033[1m\033[31mERROR: No agents found in map file\033[0m\n";
    return;
  }
  
  const auto& robot_types = robot_config["robot_types"];
  
  size_t agent_id = 0;
  for (const auto& node : map_config["agents"]) {
    std::string type = node["type"] ? node["type"].as<std::string>() : "STANDARD";
    
    if (!robot_types[type]) {
      std::cerr << "\033[1m\033[33mWARNING: Robot type '" << type 
                << "' not found";
      // Check if STANDARD exists as fallback
      if (!robot_types["STANDARD"]) {
        std::cerr << ", and STANDARD type not defined. Skipping agent.\033[0m\n";
        agent_id++;
        continue;
      }
      std::cerr << ", using STANDARD\033[0m\n";
      type = "STANDARD";
    }
    
    const auto& rt = robot_types[type];
    RobotParams params(
      rt["wheelbase"].as<double>(),
      rt["width"].as<double>(),
      rt["LF"].as<double>(),
      rt["LB"].as<double>(),
      rt["R_min"].as<double>(),
      rt["v_max"].as<double>(),
      rt["deltat"].as<double>(),
      rt["penaltyTurning"].as<double>(),
      rt["penaltyReversing"].as<double>(),
      type
    );
    
    registry.RegisterAgent(agent_id, params);
    std::cout << "Registered Agent " << agent_id << " as type " << type 
              << " (R_min=" << params.R_min << ", size=" 
              << params.LF + params.LB << "x" << params.width_W << ")\n";
    agent_id++;
  }
}

int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  std::string configFile;
  int batchSize;
  
  desc.add_options()
    ("help", "produce help message")
    ("input,i", po::value<std::string>(&inputFile)->required(),
     "input file (YAML)")
    ("output,o", po::value<std::string>(&outputFile)->required(),
     "output file (YAML)")
    ("config,c", po::value<std::string>(&configFile)
     ->default_value("../src/heterogeneous_config.yaml"),
     "robot types configuration file (YAML)")
    ("batchsize,b", po::value<int>(&batchSize)->default_value(10),
     "batch size for iter");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  std::cout << "\n=== Heterogeneous Adaptive CL-CBS ===\n\n";

  // Load robot type configurations
  YAML::Node robot_config;
  try {
    robot_config = YAML::LoadFile(configFile);
  } catch (std::exception& e) {
    std::cerr << "\033[1m\033[31mERROR: Failed to load config file: " 
              << configFile << "\033[0m\n";
    return 1;
  }

  // Load map configuration
  YAML::Node map_config;
  try {
    map_config = YAML::LoadFile(inputFile);
  } catch (std::exception& e) {
    std::cerr << "\033[1m\033[31mERROR: Failed to load map file: " 
              << inputFile << "\033[0m\n";
    return 1;
  }

  // Create fleet registry and register agents
  FleetRegistry registry;
  registerAgentsFromYAML(registry, map_config, robot_config);
  
  std::cout << "\nFleet registered with " << registry.GetNumAgents() 
            << " heterogeneous agents\n";
  
  // Sort agents by capability score (for priority-based planning)
  std::vector<std::pair<size_t, double>> agent_priorities;
  for (size_t i = 0; i < registry.GetNumAgents(); i++) {
    agent_priorities.push_back({i, registry.GetCapabilityScore(i)});
  }
  std::sort(agent_priorities.begin(), agent_priorities.end(),
            [](const auto& a, const auto& b) { return a.second > b.second; });
  
  std::cout << "\nAgent planning priority (by capability score):\n";
  for (const auto& pair : agent_priorities) {
    const auto& params = registry.GetParams(pair.first);
    std::cout << "  Agent " << pair.first << " (" << params.robot_type 
              << "): " << pair.second << "\n";
  }
  
  std::cout << "\n\033[1m\033[32mSuccessfully initialized HA-CL-CBS with heterogeneous fleet!\033[0m\n";
  std::cout << "\nNote: Full path planning integration requires extending the Environment class\n";
  std::cout << "      to use FleetRegistry for per-agent motion primitives and collision checks.\n";
  std::cout << "      This demonstration shows the FleetRegistry infrastructure is working.\n\n";

  // For now, just demonstrate that we can read and process the heterogeneous configuration
  // Full integration with CL-CBS would require modifying the Environment class extensively
  
  return 0;
}
