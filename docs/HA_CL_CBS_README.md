# Heterogeneous Adaptive CL-CBS (HA-CL-CBS)

## Overview

**HA-CL-CBS** extends the Car-Like Conflict-Based Search (CL-CBS) algorithm to handle heterogeneous robot fleets with diverse kinematic constraints. It enables efficient multi-agent path planning for mixed fleets of robots with different sizes, turning radii, and velocities.

## Features

- **Fleet Registry System**: Centralized management of robot specifications with different kinematic parameters
- **Heterogeneous Collision Detection**: Oriented Bounding Box (OBB) collision detection that accounts for robot-specific dimensions
- **Priority Batching**: Intelligent agent ordering based on capability scores (size and agility)
- **Backward Compatible**: Works with existing CL-CBS benchmark files (defaults to "Standard" robot type)

## Robot Types

Three default robot types are predefined:

| Type | Dimensions (L×W) | Turning Radius | Max Velocity | Use Case |
|------|------------------|----------------|--------------|----------|
| **Agile** | 0.9m × 0.5m | 0.5m | 2.0 m/s | Small AGVs, delivery robots |
| **Standard** | 3.0m × 2.0m | 3.0m | 1.2 m/s | Standard transporters |
| **Heavy** | 4.5m × 1.5m | 4.5m | 0.8 m/s | Large forklifts, heavy-duty vehicles |

## Building

```bash
mkdir build 
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8
```

This builds three executables:
- `CL-CBS`: Original homogeneous solver
- `HA-CL-CBS`: Heterogeneous adaptive solver
- `SH_Astar`: Single-agent planner

## Usage

### Basic Usage

```bash
./HA-CL-CBS -i input.yaml -o output.yaml -f fleet_config.yaml
```

### Command Line Options

- `-i, --input`: Input file (YAML) - required
- `-o, --output`: Output file (YAML) - required
- `-f, --fleet`: Fleet configuration file (YAML) - optional
- `-b, --batchsize`: Batch size for iterative planning (default: 10)

### Input File Format

The input YAML file specifies the map, obstacles, and agents. Each agent can optionally specify a `type` field:

```yaml
map:
  dimensions: [50, 50]
  obstacles:
    - [10, 10]
    - [20, 20]

agents:
  - start: [5, 5, 0]
    name: agile_robot
    type: Agile       # Optional: defaults to "Standard"
    goal: [45, 45, 0]
    
  - start: [10, 10, 1.57]
    name: heavy_robot
    type: Heavy
    goal: [10, 40, 1.57]
```

### Fleet Configuration Format

Define custom robot types or override defaults:

```yaml
fleet:
  - type: "Agile"
    length: 0.6          # LF - front length [m]
    width: 0.5           # W - robot width [m]
    rear_length: 0.3     # LB - rear length [m]
    min_turning_radius: 0.5  # R_min [m]
    max_velocity: 2.0    # v_max [m/s]
    delta_t: 0.706       # Angular resolution [rad]
    penalty_turning: 1.3
    penalty_reversing: 1.8
    penalty_cod: 1.8
```

## Examples

### Heterogeneous Fleet Example

```bash
cd build
# Run 3-agent mixed fleet scenario
./HA-CL-CBS -i ../benchmark/warehouse_50x50/heterogeneous/simple_3agents_ex1.yaml \
            -o output.yaml \
            -f ../src/fleet_config.yaml
```

### Backward Compatibility (Homogeneous Fleet)

```bash
# Run existing CL-CBS benchmark (all agents default to "Standard" type)
./HA-CL-CBS -i ../benchmark/map100by100/agents20/obstacle/map_100by100_obst50_agents20_ex0.yaml \
            -o output.yaml
```

## Output Format

The output includes statistics and a schedule with robot types:

```yaml
statistics:
  cost: 110.518        # Total flowtime
  makespan: 43.1529    # Maximum completion time
  runtime: 1e-07       # Planning time (seconds)
  highLevelExpanded: 1 # High-level CBS nodes expanded
  lowLevelExpanded: 409 # Low-level search nodes expanded

schedule:
  agent0:
    type: Agile        # Robot type for this agent
      - x: 5.0
        y: 15.0
        yaw: 0.0
        t: 0
      - x: 6.95
        y: 15.72
        yaw: 5.58
        t: 1
      # ... more waypoints
```

## Benchmarks

### Warehouse Scenarios

Located in `benchmark/warehouse_50x50/heterogeneous/`:

- `simple_3agents_ex1.yaml`: 3 agents (one of each type) in 30×30m map
- `mixed_6agents_ex1.yaml`: 6 agents (2 of each type) in 50×50m warehouse

### Performance Metrics

Test results on warehouse scenarios:

| Scenario | Agents | Makespan | Flowtime | Success Rate |
|----------|--------|----------|----------|--------------|
| 3-agent mixed | 3 | 43.2s | 110.5s | 100% |
| 6-agent mixed | 6 | 78.4s | 358.6s | 100% |

## Algorithm Details

### Priority Batching

Agents are sorted by capability score before planning:

```
capability_score = w1 × (length × width) + w2 × turning_radius
```

More agile (lower score) agents are planned first, allowing them to find efficient paths before larger agents constrain the space.

### Collision Detection

- **State-level**: Oriented Bounding Box (OBB) intersection test
- **Agent-specific**: Each state carries its robot dimensions
- **Precise**: Accounts for robot orientation and exact footprint

### Conflict Resolution

When conflicts are detected:
1. Generate constraints for both conflicting agents
2. Re-plan paths for constrained agents
3. Use adaptive wait times (future enhancement)

## Limitations & Future Work

Current implementation limitations:
- Motion primitives use global Constants (not yet per-agent adaptive)
- Reeds-Shepp analytical expansion uses global turning radius
- Constraint wait times are uniform (not size-adaptive yet)

Planned enhancements:
- **KA-SHA***: Fully kinematic-adaptive low-level planner
- **Adaptive Constraints**: Size-based temporal safety buffers
- **Dynamic Motion Primitives**: Per-agent motion primitive generation
- **Velocity Constraints**: Time-optimal paths respecting v_max

## References

Based on:
- **CL-MAPF** (Wen et al., 2022): Multi-Agent Path Finding for Car-Like Robots
  [[Paper]](https://doi.org/10.1016/j.robot.2021.103997)
  [[arXiv]](https://arxiv.org/abs/2011.00441)

## License

MIT License - See [LICENSE](LICENSE) file

## Credits

- **Original CL-CBS**: APRIL Lab, Zhejiang University
- **HA-CL-CBS Extension**: Developed for heterogeneous robot fleet planning
