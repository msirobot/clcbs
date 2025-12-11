# CL-CBS

## Overview

**Car-Like Conflict-Based Search (CL-CBS)** is an efficient and complete solver of Multi-Agent Path Finding for Car-like Robots problem. It applies a body conflict tree to address collisions considering the shape of agents. It also includes a new algorithm Spatiotemporal Hybrid-State A* as the single-agent path planner to generate path satisfying both kinematic and spatiotemporal constraints.

### HA-CL-CBS Extension

**Heterogeneous Adaptive CL-CBS (HA-CL-CBS)** extends CL-CBS to support **heterogeneous robot fleets** with different kinematic parameters and physical dimensions. This enables efficient multi-agent path planning for mixed fleets containing:

- **AGILE robots** (small AGVs with R_min < 1m)
- **STANDARD robots** (medium-sized robots with 1m < R_min < 3m)
- **HEAVY robots** (forklifts/trucks with R_min > 3m)

See [HA-CL-CBS Documentation](doxygen/HA-CL-CBS.md) for detailed information.

<img src="img/8car.gif" width="60%" height="60%">

The video demonstration can be found on [YouTube](https://www.youtube.com/watch?v=KThsX04ABvc)

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

* `make`: Build CL-CBS code
* `make docs`: Build doxygen documentation
* `make clang-format`: Re-format all source files
* `make all`: Build all three targets above


### Run example instances

#### CL-CBS (Homogeneous)
```bash
# make sure your are in build folder
# default 10 agent in a batch
./CL-CBS -i ../benchmark/map100by100/agents20/obstacle/map_100by100_obst50_agents20_ex13.yaml -o output.yaml 
# or compute 20 agents in a whole batch
./CL-CBS -i ../benchmark/map100by100/agents20/obstacle/map_100by100_obst50_agents20_ex13.yaml -o output.yaml -b 20 
```

#### HA-CL-CBS (Heterogeneous)
```bash
# make sure your are in build folder
./HA-CL-CBS -i ../benchmark/heterogeneous/mixed_fleet_test.yaml -o output.yaml -c ../src/heterogeneous_config.yaml
```

### Visualize Results
```bash
# make sure your are in build folder

# Visualize homogeneous CL-CBS results
python3 ../src/visualize.py -m  ../benchmark/map100by100/agents20/obstacle/map_100by100_obst50_agents20_ex13.yaml  -s output.yaml

# Visualize heterogeneous HA-CL-CBS results
cd ..
MPLBACKEND=Agg python3 src/visualize_heterogeneous.py \
    -m benchmark/heterogeneous/mixed_fleet_test.yaml \
    -c src/heterogeneous_config.yaml
```

### Agent Configuration

#### Homogeneous Agents (CL-CBS)
The agent configurations, including the size, the kinematic constraints, and penalty functions can be changed in `src/config.yaml`.

#### Heterogeneous Agents (HA-CL-CBS)
For heterogeneous fleets, define multiple robot types in `src/heterogeneous_config.yaml` and specify the type for each agent in your map file:

```yaml
agents:
  - name: agent0
    type: HEAVY
    start: [5, 5, 0]
    goal: [45, 45, 1.57]
  - name: agent1
    type: AGILE
    start: [10, 5, 0]
    goal: [40, 45, 0]
```

See [HA-CL-CBS Documentation](doxygen/HA-CL-CBS.md) for complete configuration details.

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
