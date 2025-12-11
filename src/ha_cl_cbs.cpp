/**
 * @file ha_cl_cbs.cpp
 * @brief Implementation of Heterogeneous Adaptive CL-CBS
 * @date 2024-12-11
 *
 * @copyright Copyright (c) 2024
 */
#include "ha_cl_cbs.hpp"

#include <sys/stat.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <boost/algorithm/string.hpp>
#include <boost/functional/hash.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>

#include "environment.hpp"
#include "timer.hpp"

using libMultiRobotPlanning::HA_CL_CBS;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using namespace libMultiRobotPlanning;

// Use precise collision detection for heterogeneous agents
#define PRCISE_COLLISION

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

/**
 * @brief State with agent-specific dimensions for heterogeneous collision detection
 */
struct State {
  State(double x, double y, double yaw, int time = 0,
        double length = 2.0, double width = 2.0, double rear_length = 1.0)
      : time(time), x(x), y(y), yaw(yaw),
        car_length(length), car_width(width), car_rear(rear_length) {
    rot.resize(2, 2);
    rot(0, 0) = cos(-this->yaw);
    rot(0, 1) = -sin(-this->yaw);
    rot(1, 0) = sin(-this->yaw);
    rot(1, 1) = cos(-this->yaw);
#ifdef PRCISE_COLLISION
    updateCorners();
#endif
  }

  State() = default;

  void updateCorners() {
#ifdef PRCISE_COLLISION
    // Calculate corners based on agent-specific dimensions
    corner1 = Point(
        this->x -
            sqrt(pow(car_width / 2 * 1.1, 2) + pow(car_rear * 1.1, 2)) *
                cos(atan2(car_width / 2, car_rear) - this->yaw),
        this->y -
            sqrt(pow(car_width / 2 * 1.1, 2) + pow(car_rear * 1.1, 2)) *
                sin(atan2(car_width / 2, car_rear) - this->yaw));
    corner2 = Point(
        this->x -
            sqrt(pow(car_width / 2 * 1.1, 2) + pow(car_rear * 1.1, 2)) *
                cos(atan2(car_width / 2, car_rear) + this->yaw),
        this->y +
            sqrt(pow(car_width / 2 * 1.1, 2) + pow(car_rear * 1.1, 2)) *
                sin(atan2(car_width / 2, car_rear) + this->yaw));
    corner3 = Point(
        this->x +
            sqrt(pow(car_width / 2 * 1.1, 2) + pow(car_length * 1.1, 2)) *
                cos(atan2(car_width / 2, car_length) - this->yaw),
        this->y +
            sqrt(pow(car_width / 2 * 1.1, 2) + pow(car_length * 1.1, 2)) *
                sin(atan2(car_width / 2, car_length) - this->yaw));
    corner4 = Point(
        this->x +
            sqrt(pow(car_width / 2 * 1.1, 2) + pow(car_length * 1.1, 2)) *
                cos(atan2(car_width / 2, car_length) + this->yaw),
        this->y -
            sqrt(pow(car_width / 2 * 1.1, 2) + pow(car_length * 1.1, 2)) *
                sin(atan2(car_width / 2, car_length) + this->yaw));
#endif
  }

  bool operator==(const State& s) const {
    return std::tie(time, x, y, yaw) == std::tie(s.time, s.x, s.y, s.yaw);
  }

  bool agentCollision(const State& other) const {
#ifndef PRCISE_COLLISION
    // Simple circular collision check with max dimensions
    double max_size1 = sqrt(pow(2 * car_length, 2) + pow(car_width, 2));
    double max_size2 = sqrt(pow(2 * other.car_length, 2) + pow(other.car_width, 2));
    if (pow(this->x - other.x, 2) + pow(this->y - other.y, 2) <
        pow((max_size1 + max_size2) / 2, 2))
      return true;
    return false;
#else
    // Precise OBB collision detection
    std::vector<Segment> rectangle1{Segment(this->corner1, this->corner2),
                                    Segment(this->corner2, this->corner3),
                                    Segment(this->corner3, this->corner4),
                                    Segment(this->corner4, this->corner1)};
    std::vector<Segment> rectangle2{Segment(other.corner1, other.corner2),
                                    Segment(other.corner2, other.corner3),
                                    Segment(other.corner3, other.corner4),
                                    Segment(other.corner4, other.corner1)};
    for (auto seg1 = rectangle1.begin(); seg1 != rectangle1.end(); seg1++)
      for (auto seg2 = rectangle2.begin(); seg2 != rectangle2.end(); seg2++) {
        if (boost::geometry::intersects(*seg1, *seg2)) return true;
      }
    return false;
#endif
  }

  bool obsCollision(const Location& obstacle) const {
    boost::numeric::ublas::matrix<double> obs(1, 2);
    obs(0, 0) = obstacle.x - this->x;
    obs(0, 1) = obstacle.y - this->y;

    auto rotated_obs = boost::numeric::ublas::prod(obs, rot);
    if (rotated_obs(0, 0) > -car_rear - Constants::obsRadius &&
        rotated_obs(0, 0) < car_length + Constants::obsRadius &&
        rotated_obs(0, 1) > -car_width / 2.0 - Constants::obsRadius &&
        rotated_obs(0, 1) < car_width / 2.0 + Constants::obsRadius)
      return true;
    return false;
  }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << "(" << s.x << "," << s.y << ":" << s.yaw << ")@" << s.time;
  }

  int time;
  double x;
  double y;
  double yaw;
  
  // Agent-specific dimensions
  double car_length;   // LF
  double car_width;    // W
  double car_rear;     // LB

 private:
  boost::numeric::ublas::matrix<double> rot;
  Point corner1, corner2, corner3, corner4;
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
    return seed;
  }
};
}  // namespace std

using Action = int;  // int<7 int ==6 wait

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

/**
 * @brief Extended Constraint with adaptive wait time
 */
struct Constraint {
  Constraint(int time, State s, size_t agentid, int wait_time = -1)
      : time(time), s(s), agentid(agentid), adaptive_wait_time(wait_time) {}
  Constraint() = default;
  int time;
  State s;
  size_t agentid;
  int adaptive_wait_time;  // -1 means use default

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
              << " wait:" << c.adaptive_wait_time << "]";
  }

  bool satisfyConstraint(const State& state) const {
    int wait_time = (adaptive_wait_time > 0) ? adaptive_wait_time 
                                              : Constants::constraintWaitTime;
    if (state.time < this->time || state.time > this->time + wait_time)
      return true;
    return !this->s.agentCollision(state);
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

/**
 * @brief Read fleet configuration from YAML
 */
FleetRegistry readFleetConfig(const std::string& config_file) {
  FleetRegistry fleet_registry;
  
  try {
    YAML::Node config = YAML::LoadFile(config_file);
    
    if (config["fleet"]) {
      for (const auto& robot_node : config["fleet"]) {
        RobotSpec spec;
        spec.type = robot_node["type"].as<std::string>();
        spec.length = robot_node["length"].as<double>();
        spec.width = robot_node["width"].as<double>();
        spec.rear_length = robot_node["rear_length"].as<double>();
        spec.min_turning_radius = robot_node["min_turning_radius"].as<double>();
        spec.max_velocity = robot_node["max_velocity"].as<double>();
        spec.delta_t = robot_node["delta_t"].as<double>();
        spec.penalty_turning = robot_node["penalty_turning"].as<double>();
        spec.penalty_reversing = robot_node["penalty_reversing"].as<double>();
        spec.penalty_cod = robot_node["penalty_cod"].as<double>();
        
        fleet_registry.registerRobotSpec(spec);
      }
    }
  } catch (std::exception& e) {
    std::cerr << "\033[1m\033[33mWARNING: Could not load fleet config: "
              << config_file << "\033[0m\n";
    std::cerr << "Using default fleet specifications.\n";
  }
  
  return fleet_registry;
}

/**
 * @brief Read agent configuration from file (backward compatible)
 */
void readAgentConfig() {
  YAML::Node car_config;
  std::string test(__FILE__);
  boost::replace_all(test, "ha_cl_cbs.cpp", "config.yaml");
  try {
    car_config = YAML::LoadFile(test.c_str());
  } catch (std::exception& e) {
    std::cerr << "\033[1m\033[33mWARNING: Failed to load agent config file: "
              << test << "\033[0m , Using default params. \n";
  }
  
  // Set default (Standard robot) parameters for backward compatibility
  Constants::r = car_config["r"] ? car_config["r"].as<double>() : 3.0;
  Constants::deltat = car_config["deltat"] ? car_config["deltat"].as<double>() : 0.706;
  Constants::penaltyTurning = car_config["penaltyTurning"] ? 
                              car_config["penaltyTurning"].as<double>() : 1.5;
  Constants::penaltyReversing = car_config["penaltyReversing"] ?
                                car_config["penaltyReversing"].as<double>() : 2.0;
  Constants::penaltyCOD = car_config["penaltyCOD"] ?
                          car_config["penaltyCOD"].as<double>() : 2.0;
  Constants::mapResolution = car_config["mapResolution"] ?
                             car_config["mapResolution"].as<double>() : 2.0;
  Constants::xyResolution = Constants::r * Constants::deltat;
  Constants::yawResolution = Constants::deltat;

  Constants::carWidth = car_config["carWidth"] ?
                        car_config["carWidth"].as<double>() : 2.0;
  Constants::LF = car_config["LF"] ? car_config["LF"].as<double>() : 2.0;
  Constants::LB = car_config["LB"] ? car_config["LB"].as<double>() : 1.0;
  Constants::obsRadius = car_config["obsRadius"] ?
                         car_config["obsRadius"].as<double>() : 0.8;
  Constants::constraintWaitTime = car_config["constraintWaitTime"] ?
                                  car_config["constraintWaitTime"].as<double>() : 2;

  Constants::dx = {Constants::r * Constants::deltat,
                   Constants::r * sin(Constants::deltat),
                   Constants::r * sin(Constants::deltat),
                   -Constants::r * Constants::deltat,
                   -Constants::r * sin(Constants::deltat),
                   -Constants::r * sin(Constants::deltat)};
  Constants::dy = {0,
                   -Constants::r * (1 - cos(Constants::deltat)),
                   Constants::r * (1 - cos(Constants::deltat)),
                   0,
                   -Constants::r * (1 - cos(Constants::deltat)),
                   Constants::r * (1 - cos(Constants::deltat))};
  Constants::dyaw = {0, Constants::deltat,  -Constants::deltat,
                     0, -Constants::deltat, Constants::deltat};
}

int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  std::string fleetConfig;
  int batchSize;
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (YAML)")("output,o",
                           po::value<std::string>(&outputFile)->required(),
                           "output file (YAML)")(
      "fleet,f", po::value<std::string>(&fleetConfig)->default_value(""),
      "fleet configuration file (YAML)")(
      "batchsize,b", po::value<int>(&batchSize)->default_value(10),
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

  readAgentConfig();
  
  // Load fleet registry
  FleetRegistry fleet_registry;
  if (!fleetConfig.empty()) {
    fleet_registry = readFleetConfig(fleetConfig);
  }

  YAML::Node map_config;
  try {
    map_config = YAML::LoadFile(inputFile);
  } catch (std::exception& e) {
    std::cerr << "\033[1m\033[31mERROR: Failed to load map file: " << inputFile
              << "\033[0m \n";
    return 0;
  }
  
  const auto& dim = map_config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  std::unordered_set<Location> obstacles;
  std::multimap<int, State> dynamic_obstacles;
  std::vector<State> goals;
  std::vector<State> startStates;
  std::vector<std::string> agent_types;
  
  for (const auto& node : map_config["map"]["obstacles"]) {
    obstacles.insert(Location(node[0].as<double>(), node[1].as<double>()));
  }
  
  for (const auto& node : map_config["agents"]) {
    const auto& start = node["start"];
    const auto& goal = node["goal"];
    
    // Get agent type (default to "Standard" for backward compatibility)
    std::string type = "Standard";
    if (node["type"]) {
      type = node["type"].as<std::string>();
    }
    agent_types.push_back(type);
    
    const RobotSpec& spec = fleet_registry.getRobotSpec(type);
    
    startStates.emplace_back(State(start[0].as<double>(), start[1].as<double>(),
                                   start[2].as<double>(), 0,
                                   spec.length, spec.width, spec.rear_length));
    goals.emplace_back(State(goal[0].as<double>(), goal[1].as<double>(),
                             goal[2].as<double>(), 0,
                             spec.length, spec.width, spec.rear_length));
  }

  std::cout << "Calculating HA-CL-CBS Solution with " << agent_types.size() 
            << " heterogeneous agents...\n";
  
  // Print fleet composition
  std::map<std::string, int> type_counts;
  for (const auto& type : agent_types) {
    type_counts[type]++;
  }
  std::cout << "Fleet composition:\n";
  for (const auto& pair : type_counts) {
    std::cout << "  " << pair.first << ": " << pair.second << " agents\n";
  }
  
  double timer = 0;
  bool success = false;
  std::vector<PlanResult<State, Action, double>> solution;
  
  // For now, process all agents in one batch
  // TODO: Implement batching for large fleets
  
  Environment<Location, State, Action, double, Conflict, Constraint,
              Constraints>
      mapf(dimx, dimy, obstacles, dynamic_obstacles, goals);
      
  if (!mapf.startAndGoalValid(startStates, 0, startStates.size())) {
    std::cerr << "\033[1m\033[31mERROR: Invalid start/goal configuration\033[0m\n";
    return 1;
  }

  HA_CL_CBS<State, Action, double, Conflict, Constraints,
            Environment<Location, State, Action, double, Conflict, Constraint,
                       Constraints>>
      ha_cbs(mapf, fleet_registry);

  Timer planning_timer;
  success = ha_cbs.search(startStates, agent_types, solution);
  timer = planning_timer.elapsedSeconds();

  if (success) {
    std::cout << "\033[1m\033[32m Successfully find solution! \033[0m\n";
    std::cout << "Planning time: " << timer << " s\n";
    
    // Calculate statistics
    double total_flowtime = 0;
    double max_makespan = 0;
    for (const auto& s : solution) {
      total_flowtime += s.cost;
      max_makespan = std::max(max_makespan, s.cost);
    }
    
    std::cout << "Makespan: " << max_makespan << "\n";
    std::cout << "Flowtime: " << total_flowtime << "\n";
    std::cout << "High-level expanded: " << mapf.highLevelExpanded() << "\n";
    std::cout << "Low-level expanded: " << mapf.lowLevelExpanded() << "\n";

    // Write output
    std::ofstream out(outputFile);
    out << "statistics:" << std::endl;
    out << "  cost: " << total_flowtime << std::endl;
    out << "  makespan: " << max_makespan << std::endl;
    out << "  runtime: " << timer << std::endl;
    out << "  highLevelExpanded: " << mapf.highLevelExpanded() << std::endl;
    out << "  lowLevelExpanded: " << mapf.lowLevelExpanded() << std::endl;
    out << "schedule:" << std::endl;
    for (size_t a = 0; a < solution.size(); ++a) {
      out << "  agent" << a << ":" << std::endl;
      out << "    type: " << agent_types[a] << std::endl;
      for (const auto& state : solution[a].states) {
        out << "      - x: " << state.first.x << std::endl
            << "        y: " << state.first.y << std::endl
            << "        yaw: " << state.first.yaw << std::endl
            << "        t: " << state.first.time << std::endl;
      }
    }
  } else {
    std::cout << "\033[1m\033[31m Fail to find paths \033[0m\n";
  }

  return 0;
}
