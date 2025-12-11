# Heterogeneous Adaptive CL-CBS (HA-CL-CBS)

## Overview

**HA-CL-CBS** extends the Car-Like Conflict-Based Search (CL-CBS) algorithm to support **heterogeneous robot fleets** with different kinematic parameters and physical dimensions. This enables efficient multi-agent path planning for mixed fleets of robots with varying capabilities.

## Key Features

### 1. Heterogeneous Fleet Support

HA-CL-CBS supports multiple robot types in a single planning instance:

- **AGILE Robots**: Small, highly maneuverable robots (e.g., small AGVs)
  - Low turning radius (R_min < 1m)
  - High velocity
  - Compact dimensions

- **STANDARD Robots**: Medium-sized robots with balanced capabilities
  - Moderate turning radius (1m < R_min < 3m)
  - Moderate velocity
  - Medium dimensions

- **HEAVY Robots**: Large, less maneuverable robots (e.g., forklifts, trucks)
  - Large turning radius (R_min > 3m)
  - Lower velocity
  - Large dimensions

### 2. Fleet Registry System

The `FleetRegistry` class manages per-agent kinematic parameters:

```cpp
struct RobotParams {
    double wheelbase_L;      // Wheelbase length
    double width_W;          // Robot width
    double length;           // Total length (LF + LB)
    double LF;               // Front overhang
    double LB;               // Rear overhang
    double R_min;            // Minimum turning radius
    double v_max;            // Maximum velocity
    double delta_t;          // Steering angle resolution
    double penaltyTurning;   // Turning cost penalty
    double penaltyReversing; // Reversing cost penalty
    std::string robot_type;  // Type identifier
};
```

### 3. Capability-Based Priority Planning

Agents are planned in order of their capability score:

```
c_i = 0.6 × (L_i × W_i) + 0.4 × R_min_i
```

- Higher score = larger/less maneuverable = planned first
- Ensures constrained robots get priority in path selection
- Reduces conflicts and replanning iterations

### 4. Adaptive Collision Detection

- Uses actual robot dimensions for collision checking
- Oriented Bounding Box (OBB) collision detection
- Agent-specific safety margins based on robot size

## Configuration

### Robot Type Configuration

Define robot types in `heterogeneous_config.yaml`:

```yaml
robot_types:
  AGILE:
    wheelbase: 0.5
    width: 0.5
    LF: 0.4
    LB: 0.2
    R_min: 0.5
    v_max: 2.0
    deltat: 0.15
    penaltyTurning: 1.2
    penaltyReversing: 1.5
    
  STANDARD:
    wheelbase: 1.2
    width: 1.0
    LF: 1.0
    LB: 0.5
    R_min: 2.5
    v_max: 1.2
    deltat: 0.12
    penaltyTurning: 1.5
    penaltyReversing: 2.0
    
  HEAVY:
    wheelbase: 2.5
    width: 1.5
    LF: 2.0
    LB: 1.0
    R_min: 4.5
    v_max: 0.8
    deltat: 0.08
    penaltyTurning: 2.0
    penaltyReversing: 2.5
```

### Map File with Heterogeneous Agents

Specify agent types in your map file:

```yaml
map:
  dimensions: [50, 50]
  obstacles:
    - [10, 10]
    - [20, 20]

agents:
  - name: agent0
    type: HEAVY
    start: [5, 5, 0]
    goal: [45, 45, 1.57]
  
  - name: agent1
    type: AGILE
    start: [10, 5, 0]
    goal: [40, 45, 0]
  
  - name: agent2
    type: STANDARD
    start: [15, 5, 0.5]
    goal: [35, 40, -1.57]
```

## Usage

### Build

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make HA-CL-CBS
```

### Run

```bash
./HA-CL-CBS -i ../benchmark/heterogeneous/mixed_fleet_test.yaml \
            -o output.yaml \
            -c ../src/heterogeneous_config.yaml
```

### Command-Line Options

- `-i, --input`: Input map file (YAML) with agent start/goal positions and types
- `-o, --output`: Output solution file (YAML)
- `-c, --config`: Robot types configuration file (default: `../src/heterogeneous_config.yaml`)
- `-b, --batchsize`: Batch size for iterative planning (default: 10)

## Algorithm Details

### Kinematic-Adaptive Motion Primitives

Motion primitives are selected based on the robot's minimum turning radius:

- **R_min < 1.0m** → AGILE primitives (finer steering angles)
- **1.0m ≤ R_min ≤ 3.0m** → STANDARD primitives
- **R_min > 3.0m** → HEAVY primitives (coarser, wider turns)

For each primitive:
- `dx = R_min × δθ` (for straight motion)
- `dx = R_min × sin(δθ)` (for turns)
- `dy = ±R_min × (1 - cos(δθ))`

### Adaptive Safety Buffers

Safety margins scale with robot size:

```
δt_i = base_buffer × (1 + 0.5 × (dimensions_i / max_dimensions))
```

## Implementation Status

### Completed Components

- ✅ Fleet Registry system (`include/fleet_registry.hpp`)
- ✅ Heterogeneous State class (`include/heterogeneous_state.hpp`)
- ✅ Robot type configuration file (`src/heterogeneous_config.yaml`)
- ✅ Example benchmark file (`benchmark/heterogeneous/mixed_fleet_test.yaml`)
- ✅ Main executable with configuration loading (`src/ha_cl_cbs.cpp`)
- ✅ Priority-based agent sorting by capability score
- ✅ CMake build integration

### Future Enhancements

- 🔲 Full integration with CL-CBS Environment class
- 🔲 Per-agent motion primitive generation
- 🔲 Agent-specific Reeds-Shepp heuristic calculation
- 🔲 Heterogeneous collision constraint generation
- 🔲 Visualization with dimension-based rendering

## Architecture

```
┌─────────────────────┐
│  HA-CL-CBS Main     │
│  (ha_cl_cbs.cpp)    │
└──────────┬──────────┘
           │
           ├─► FleetRegistry
           │   (fleet_registry.hpp)
           │   - Stores robot parameters
           │   - Calculates capability scores
           │
           ├─► HeterogeneousState
           │   (heterogeneous_state.hpp)
           │   - Agent-specific collision
           │   - Dimension-aware checks
           │
           └─► CL-CBS Algorithm
               (cl_cbs.hpp)
               - High-level CBS search
               - Low-level hybrid A*
```

## Example Scenarios

### Mixed Fleet Warehouse

- 3 AGILE robots for item picking
- 2 STANDARD robots for pallet transport
- 1 HEAVY forklift for bulk operations

Benefits:
- HEAVY robot planned first, gets optimal wide-radius path
- AGILE robots navigate around constraints efficiently
- Reduced total makespan compared to homogeneous planning

## References

This implementation extends the work described in:

```
@article{WEN2022103997,
    title = {CL-MAPF: Multi-Agent Path Finding for Car-Like robots with 
             kinematic and spatiotemporal constraints},
    journal = {Robotics and Autonomous Systems},
    volume = {150},
    pages = {103997},
    year = {2022},
    author = {Licheng Wen and Yong Liu and Hongliang Li},
}
```

## License

MIT License - See LICENSE file for details.
