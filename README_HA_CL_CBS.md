# HA-CL-CBS: Heterogeneous-Adaptive Car-Like Conflict-Based Search

## Overview

HA-CL-CBS extends the existing CL-CBS (Car-Like Conflict-Based Search) implementation to support **heterogeneous robot fleets** where each robot can have different kinematic parameters (size, turning radius, velocity).

## Implementation Status

### ✅ Completed Components

1. **Core Data Structures** (`include/agent_params.hpp`)
   - `AgentParams` structure storing per-agent kinematic parameters
   - Automatic computation of derived parameters (motion primitives, turning radius)
   - Methods for retrieving agent-specific dx, dy, dyaw values

2. **Fleet Registry** (`include/fleet_registry.hpp`)
   - Central registry for managing heterogeneous fleet
   - YAML-based configuration loading
   - Priority-based agent sorting (capability score computation)
   - Fleet summary and validation

3. **Collision Detection** (`include/geometry_collision.hpp`)
   - Oriented Bounding Box (OBB) implementation
   - Separating Axis Theorem (SAT) for OBB-OBB collision detection
   - Agent-to-obstacle collision checking with OBB
   - Actual robot dimensions used for accurate collision detection

4. **Environment Extension** (`include/ha_environment.hpp`)
   - HAState, HAConflict, HAConstraint structures
   - HAEnvironment class with fleet registry integration
   - Agent-specific heuristics using Reeds-Shepp with custom turning radius
   - Heterogeneous collision checking in getFirstConflict()

5. **HA-CL-CBS Solver** (`include/ha_cl_cbs.hpp`)
   - Template class following CL-CBS pattern
   - Priority-based planning order (larger/stiffer robots first)
   - High-level CBS search with agent-specific constraints
   - Low-level wrapper for Hybrid A* planning

6. **Main Entry Point** (`src/ha_cl_cbs.cpp`)
   - Command-line interface (`-i`, `-o`, `-f` flags)
   - YAML input/output
   - Default heterogeneous fleet generation
   - Statistics reporting

7. **Configuration Files**
   - `config/fleet_registry.yaml` - Full 6-agent heterogeneous fleet
   - `config/simple_fleet.yaml` - 3-agent test configuration
   - `config/single_agent.yaml` - Single agent for testing

8. **Test Scenarios**
   - `benchmark/heterogeneous_test.yaml` - 6 agents with obstacles
   - `benchmark/simple_hetero_test.yaml` - 3 agents simple scenario
   - `benchmark/minimal_test.yaml` - Single agent test

9. **Build System**
   - Updated `CMakeLists.txt` with `ha_cl_cbs` executable target
   - Created `cmake/FindOMPL.cmake` module for dependency resolution
   - Added `.gitignore` for build artifacts

10. **Visualization Support** (`src/visualize.py`)
    - Updated to support agent-specific dimensions (basic implementation)
    - Infrastructure for heterogeneous robot rendering

### ⚠️ Known Issues

**Low-Level Planner Not Terminating**: The HA-CL-CBS executable builds successfully and correctly initializes the fleet registry, but the low-level path planner (Hybrid A*) does not terminate when searching for individual agent paths. This appears to be related to one of:

1. **isSolution Implementation**: The simplified goal checking in HAEnvironment may not correctly implement the analytic expansion pattern expected by Hybrid A* (which uses Reeds-Shepp curves to connect to the goal)

2. **State Indexing**: The `calcIndex()` function in HAEnvironment may have issues with hash collisions or state space discretization

3. **State Validation**: The `stateValid()` checks may be too restrictive or have bugs

### Recommended Next Steps to Fix

1. **Add Analytic Expansion**: Implement full Reeds-Shepp analytic expansion in `isSolution()` similar to the original Environment (lines 240-322 of environment.hpp)

2. **Add Debug Logging**: Insert debug output in:
   - `HAEnvironment::isSolution()` - to see if it's ever reached
   - `HAEnvironment::getNeighbors()` - to verify state expansion
   - `HybridAStar` search loop - to see if states are being explored

3. **Simplify for Testing**: Start with homogeneous parameters matching the original Constants to isolate the issue

4. **Verify State Hashing**: Ensure `std::hash<HAState>` produces good distribution

## Usage

```bash
# Build
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make ha_cl_cbs

# Run with custom fleet
./ha_cl_cbs -i ../benchmark/heterogeneous_test.yaml \
            -o output.yaml \
            -f ../config/fleet_registry.yaml

# Run with default heterogeneous fleet
./ha_cl_cbs -i ../benchmark/heterogeneous_test.yaml \
            -o output.yaml
```

## Fleet Configuration Format

```yaml
agents:
  - id: 0
    type: "SORTING_AGV"
    length: 0.6        # meters
    width: 0.5         # meters
    wheelbase: 0.4     # meters
    LF: 0.35           # front overhang
    LB: 0.25           # rear overhang
    max_steering_angle: 0.8  # radians
    max_velocity: 2.0  # m/s
    penalty_turning: 1.3
    penalty_reversing: 2.0
```

## Architecture

### Priority-Based Planning

Agents are planned in order of decreasing capability score:
```
capability_score = 0.3 * (length × width) + 0.7 × min_turning_radius
```

Larger/stiffer robots are planned first to reduce conflicts.

### Collision Detection

- **OBB (Oriented Bounding Box)**: Each robot represented as rotated rectangle
- **SAT (Separating Axis Theorem)**: Accurate polygon-polygon intersection
- **Agent Dimensions**: Actual length/width used (not fixed constants)

### Motion Primitives

Each agent uses its own motion primitives based on:
- `min_turning_radius` = wheelbase / tan(max_steering_angle)
- Discretization angle: `deltat = 0.706` radians
- 6 actions: forward-straight, forward-left, forward-right, backward-straight, backward-left, backward-right

## Key Design Decisions

1. **Minimal Changes**: Extended existing structures rather than replacing them
2. **Backward Compatibility**: Original CL-CBS still works unchanged
3. **Flexibility**: Fleet configuration via YAML for easy testing
4. **Priority Planning**: Implicit coordination through planning order
5. **Accurate Collision**: OBB-based detection using actual dimensions

## Testing

### Verification with Original CL-CBS

```bash
# Test that original CL-CBS still works
./CL-CBS -i ../benchmark/simple_hetero_test.yaml -o test_output.yaml

# Result: ✅ Successfully finds solution in ~3 seconds
```

### Unit Testing Approach

1. Single agent path finding
2. Two-agent conflict detection
3. Multi-agent coordination
4. Different robot type combinations

## Files Modified

- `CMakeLists.txt` - Added ha_cl_cbs target, cmake module path
- `src/cl_cbs.cpp` - Added boost/algorithm/string.hpp include
- `src/visualize.py` - Basic support for heterogeneous dimensions
- `.gitignore` - Added (new file)

## Files Created

### Headers (include/)
- `agent_params.hpp` - Agent parameter structure
- `fleet_registry.hpp` - Fleet management
- `geometry_collision.hpp` - OBB collision detection
- `ha_environment.hpp` - Heterogeneous environment
- `ha_cl_cbs.hpp` - HA-CL-CBS solver

### Source (src/)
- `ha_cl_cbs.cpp` - Main entry point

### Configuration (config/)
- `fleet_registry.yaml` - 6-agent fleet
- `simple_fleet.yaml` - 3-agent fleet
- `single_agent.yaml` - 1-agent fleet

### Test Scenarios (benchmark/)
- `heterogeneous_test.yaml` - Full test
- `simple_hetero_test.yaml` - Simple test
- `minimal_test.yaml` - Minimal test

### Build System (cmake/)
- `FindOMPL.cmake` - OMPL dependency resolution

## Future Enhancements

1. **Complete Low-Level Planner**: Fix Hybrid A* termination issue
2. **Advanced Visualization**: Render different robot sizes/colors
3. **Dynamic Fleet Updates**: Add/remove agents at runtime
4. **Velocity Constraints**: Use max_velocity in planning
5. **Path Smoothing**: Post-process paths for smoother motion
6. **Benchmark Suite**: Comprehensive heterogeneous scenarios
7. **Performance Metrics**: Compare to homogeneous baseline

## References

- Original CL-CBS paper: Wen et al., "CL-MAPF: Multi-Agent Path Finding for Car-Like robots", RAS 2022
- OMPL library: https://ompl.kavrakilab.org/
- Reeds-Shepp curves for car-like motion planning
