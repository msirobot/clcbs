# CL-CBS & HA-CL-CBS

## Overview

This repository contains two multi-agent path planning algorithms for car-like robots:

1. **CL-CBS (Car-Like Conflict-Based Search)**: Efficient and complete solver for homogeneous fleets
2. **HA-CL-CBS (Heterogeneous Adaptive CL-CBS)**: Extended solver for heterogeneous robot fleets with diverse kinematic constraints

### CL-CBS

**Car-Like Conflict-Based Search (CL-CBS)** is an efficient and complete solver of Multi-Agent Path Finding for Car-like Robots problem. It applies a body conflict tree to address collisions considering the shape of agents. It also includes a new algorithm Spatiotemporal Hybrid-State A* as the single-agent path planner to generate path satisfying both kinematic and spatiotemporal constraints.

<img src="img/8car.gif" width="60%" height="60%">

The video demonstration can be found on [YouTube](https://www.youtube.com/watch?v=KThsX04ABvc)

### HA-CL-CBS (NEW!)

**Heterogeneous Adaptive CL-CBS** extends CL-CBS to handle mixed robot fleets with different:
- **Sizes**: From small AGVs (0.9m × 0.5m) to large forklifts (4.5m × 1.5m)
- **Turning Radii**: From agile robots (R_min = 0.5m) to heavy vehicles (R_min = 4.5m)
- **Velocities**: From slow heavy-duty (0.8 m/s) to fast delivery robots (2.0 m/s)

**Key Features:**
- Fleet Registry System for managing robot specifications
- Oriented Bounding Box (OBB) collision detection for heterogeneous agents
- Priority batching based on capability scores
- Backward compatible with existing CL-CBS scenarios

📖 **[Read the full HA-CL-CBS documentation](docs/HA_CL_CBS_README.md)**

## Source Code
### Requirement
```bash
sudo apt-get install g++ cmake libboost-program-options-dev libyaml-cpp-dev \
clang-tidy clang-format python3-matplotlib libompl-dev libeigen3-dev
```
> Note: Please make sure your `matplotlib` version is above `2.0`, otherwise it may show weird image while visualization. You can upgrade it by `pip3 install -U matplotlib`.


### Build
```bash
mkdir build 
cd build
cmake -DCMAKE_BUILD_TYPE=Release  ..
make -j8
```

This builds three executables:
* `CL-CBS`: Original homogeneous solver
* `HA-CL-CBS`: Heterogeneous adaptive solver
* `SH_Astar`: Single-agent planner

### Build Targets
* `make`: Build all executables
* `make docs`: Build doxygen documentation
* `make clang-format`: Re-format all source files


### Run CL-CBS (Homogeneous Fleet)
```bash
# make sure you are in build folder
# default 10 agents in a batch
./CL-CBS -i ../benchmark/map100by100/agents20/obstacle/map_100by100_obst50_agents20_ex13.yaml -o output.yaml 
# or compute 20 agents in a whole batch
./CL-CBS -i ../benchmark/map100by100/agents20/obstacle/map_100by100_obst50_agents20_ex13.yaml -o output.yaml -b 20 
```

### Run HA-CL-CBS (Heterogeneous Fleet)
```bash
# make sure you are in build folder
# Run 3-agent mixed fleet scenario
./HA-CL-CBS -i ../benchmark/warehouse_50x50/heterogeneous/simple_3agents_ex1.yaml \
            -o output.yaml \
            -f ../src/fleet_config.yaml

# Run 6-agent mixed fleet scenario
./HA-CL-CBS -i ../benchmark/warehouse_50x50/heterogeneous/mixed_6agents_ex1.yaml \
            -o output.yaml \
            -f ../src/fleet_config.yaml

# Backward compatibility: run with existing benchmarks (defaults to Standard type)
./HA-CL-CBS -i ../benchmark/map100by100/agents20/obstacle/map_100by100_obst50_agents20_ex0.yaml \
            -o output.yaml
```

### Visualize Results
```bash
# make sure your are in build folder
python3 ../src/visualize.py -m  ../benchmark/map100by100/agents20/obstacle/map_100by100_obst50_agents20_ex13.yaml  -s output.yaml
```

### Agent Configuration
The agent configurations, including the size, the kinematic constraints, and penalty functions can be changed in `src/config.yaml`.

## Benchmark

Benchmark for evaluating CL-MAPF problem are available in `benchmark` folder. It contains 3000 unique instances with different map size and agents number.

The folder are arranged like follows, each mapset contains 60 instances:

```
benchmark
├── map100by100
│   ├── agents10
│   │   ├── empty
│   │   └── obstacle
│   ...
├── map300by300
│   ├── agents10
│   │   ├── empty
│   │   └── obstacle
│   ...
└── map50by50
    ├── agents10
    │   ├── empty
    │   └── obstacle
    ...
```

The instance are in `yaml` format.

A typical result from benchmark acts like below:

<img src="img/dataset.gif" width="60%" height="60%">

## Credits 

For researchers that have leveraged or compared to this work, please cite the following:

```
@article{WEN2022103997,
    title = {CL-MAPF: Multi-Agent Path Finding for Car-Like robots with kinematic and spatiotemporal constraints},
    journal = {Robotics and Autonomous Systems},
    volume = {150},
    pages = {103997},
    year = {2022},
    issn = {0921-8890},
    doi = {https://doi.org/10.1016/j.robot.2021.103997},
    url = {https://www.sciencedirect.com/science/article/pii/S0921889021002530},
    author = {Licheng Wen and Yong Liu and Hongliang Li},
}
```


## License
The code was developed by the  [APRIL Lab](https://github.com/APRIL-ZJU) in Zhejiang University, and is provided under the [MIT License](https://opensource.org/licenses/MIT).
