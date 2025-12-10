/**
 * @file ha_cl_cbs.cpp
 * @brief Main entry point for HA-CL-CBS
 * @date 2024-12-10
 */

#include "ha_cl_cbs.hpp"

#include <yaml-cpp/yaml.h>

#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>

#include "timer.hpp"

using libMultiRobotPlanning::HA_CL_CBS;
using libMultiRobotPlanning::FleetRegistry;
using libMultiRobotPlanning::AgentParams;
using libMultiRobotPlanning::HAEnvironment;
using libMultiRobotPlanning::PlanResult;

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

using Action = int;
using State = libMultiRobotPlanning::HAState<Location>;

/**
 * @brief Create default heterogeneous fleet if no config provided
 */
void createDefaultFleet(FleetRegistry& registry, size_t num_agents) {
  std::cout << "Creating default heterogeneous fleet...\n";

  for (size_t i = 0; i < num_agents; ++i) {
    AgentParams params;
    params.agent_id = i;

    // Assign different types based on agent index
    if (i % 3 == 0) {
      // Type A: Small AGV
      params.agent_type = "SORTING_AGV";
      params.length = 0.6;
      params.width = 0.5;
      params.wheelbase = 0.4;
      params.LF = 0.35;
      params.LB = 0.25;
      params.max_steering_angle = 0.8;
      params.max_velocity = 2.0;
    } else if (i % 3 == 1) {
      // Type B: Medium Transporter
      params.agent_type = "TRANSPORTER";
      params.length = 1.5;
      params.width = 1.0;
      params.wheelbase = 1.2;
      params.LF = 0.9;
      params.LB = 0.6;
      params.max_steering_angle = 0.5;
      params.max_velocity = 1.2;
    } else {
      // Type C: Heavy Forklift
      params.agent_type = "HEAVY_FORKLIFT";
      params.length = 3.0;
      params.width = 1.5;
      params.wheelbase = 2.5;
      params.LF = 2.0;
      params.LB = 1.0;
      params.max_steering_angle = 0.35;
      params.max_velocity = 0.8;
    }

    params.min_turning_radius =
        params.wheelbase / std::tan(params.max_steering_angle);
    params.computeDerivedParams();

    registry.registerAgent(params);
  }
}

int main(int argc, char* argv[]) {
  namespace po = boost::program_options;

  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  std::string fleetConfigFile;

  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (YAML)")("output,o",
                           po::value<std::string>(&outputFile)->required(),
                           "output file (YAML)")(
      "fleet,f", po::value<std::string>(&fleetConfigFile),
      "fleet configuration file (YAML)");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
      std::cout << desc << "\n";
      return 0;
    }

    po::notify(vm);
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  // Load map configuration
  YAML::Node map_config;
  try {
    map_config = YAML::LoadFile(inputFile);
  } catch (std::exception& e) {
    std::cerr << "\033[1m\033[31mERROR: Failed to load map file: " << inputFile
              << "\033[0m\n";
    return 1;
  }

  const auto& dim = map_config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  std::unordered_set<Location> obstacles;
  std::multimap<int, State> dynamic_obstacles;
  std::vector<State> goals;
  std::vector<State> startStates;

  for (const auto& node : map_config["map"]["obstacles"]) {
    obstacles.insert(Location(node[0].as<double>(), node[1].as<double>()));
  }

  for (const auto& node : map_config["agents"]) {
    const auto& start = node["start"];
    const auto& goal = node["goal"];
    startStates.emplace_back(State(start[0].as<double>(), start[1].as<double>(),
                                   start[2].as<double>()));
    goals.emplace_back(State(goal[0].as<double>(), goal[1].as<double>(),
                             goal[2].as<double>()));
  }

  // Load or create fleet registry
  FleetRegistry fleet_registry;
  if (!fleetConfigFile.empty()) {
    std::cout << "Loading fleet configuration from: " << fleetConfigFile
              << "\n";
    if (!fleet_registry.loadFromYAML(fleetConfigFile)) {
      std::cerr << "Failed to load fleet config, using default\n";
      createDefaultFleet(fleet_registry, goals.size());
    }
  } else {
    createDefaultFleet(fleet_registry, goals.size());
  }

  fleet_registry.printSummary();

  // Create environment
  HAEnvironment<Location, Action, double> mapf(dimx, dimy, obstacles,
                                                dynamic_obstacles, goals,
                                                fleet_registry);

  // Run HA-CL-CBS
  HA_CL_CBS<Location, Action, double> solver(mapf, fleet_registry);

  std::cout << "\nCalculating Solution...\n";
  libMultiRobotPlanning::Timer timer;
  std::vector<PlanResult<State, Action, double>> solution;
  bool success = solver.search(startStates, solution);
  timer.stop();

  std::ofstream out(outputFile);

  if (success) {
    std::cout << "\033[1m\033[32m Successfully found solution! \033[0m\n";

    double makespan = 0, flowtime = 0, cost = 0;
    for (const auto& s : solution) cost += s.cost;

    for (size_t a = 0; a < solution.size(); ++a) {
      double current_makespan = 0;
      for (size_t i = 0; i < solution[a].actions.size(); ++i) {
        const auto& params = fleet_registry.getParams(a);
        if (solution[a].actions[i].first % 3 == 0)
          current_makespan += params.getDx(0);
        else
          current_makespan += params.min_turning_radius * params.deltat;
      }
      flowtime += current_makespan;
      if (current_makespan > makespan) makespan = current_makespan;
    }

    std::cout << " Runtime: " << timer.elapsedSeconds() << "s\n"
              << " Makespan: " << makespan << "\n"
              << " Flowtime: " << flowtime << "\n"
              << " Cost: " << cost << "\n"
              << " High-level expanded: " << mapf.highLevelExpanded() << "\n"
              << " Low-level expanded: " << mapf.lowLevelExpanded() << "\n";

    // Output to file
    out << "statistics:" << std::endl;
    out << "  cost: " << cost << std::endl;
    out << "  makespan: " << makespan << std::endl;
    out << "  flowtime: " << flowtime << std::endl;
    out << "  runtime: " << timer.elapsedSeconds() << std::endl;
    out << "schedule:" << std::endl;

    for (size_t a = 0; a < solution.size(); ++a) {
      out << "  agent" << a << ":" << std::endl;
      const auto& params = fleet_registry.getParams(a);
      out << "    type: " << params.agent_type << std::endl;
      out << "    length: " << params.length << std::endl;
      out << "    width: " << params.width << std::endl;
      out << "    states:" << std::endl;
      for (const auto& state : solution[a].states) {
        out << "      - x: " << state.first.x << std::endl
            << "        y: " << state.first.y << std::endl
            << "        yaw: " << state.first.yaw << std::endl
            << "        t: " << state.first.time << std::endl;
      }
    }
  } else {
    std::cout << "\033[1m\033[31m Failed to find solution \033[0m\n";
  }

  return success ? 0 : 1;
}
