#!/usr/bin/env python3
"""
Heterogeneous Robot Fleet Visualization for HA-CL-CBS
Extends the original visualize.py to support robots with different dimensions
"""

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import yaml
import argparse
import numpy as np

# Default robot parameters (can be overridden by config)
ROBOT_TYPES = {
    'AGILE': {'width': 0.5, 'LF': 0.4, 'LB': 0.2, 'color': 'green'},
    'STANDARD': {'width': 1.0, 'LF': 1.0, 'LB': 0.5, 'color': 'blue'},
    'HEAVY': {'width': 1.5, 'LF': 2.0, 'LB': 1.0, 'color': 'red'}
}

def load_config(config_file):
    """Load robot type configurations from YAML"""
    try:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
            if 'robot_types' in config:
                for rtype, params in config['robot_types'].items():
                    if rtype not in ROBOT_TYPES:
                        ROBOT_TYPES[rtype] = {'color': 'blue'}  # Default color
                    ROBOT_TYPES[rtype].update({
                        'width': params.get('width', 1.0),
                        'LF': params.get('LF', 1.0),
                        'LB': params.get('LB', 0.5),
                        'R_min': params.get('R_min', 2.5)
                    })
    except Exception as e:
        print(f"Warning: Could not load config file {config_file}: {e}")

def draw_robot(ax, x, y, yaw, robot_type='STANDARD', alpha=1.0):
    """Draw a robot with its specific dimensions"""
    params = ROBOT_TYPES.get(robot_type, ROBOT_TYPES['STANDARD'])
    width = params['width']
    LF = params['LF']
    LB = params['LB']
    color = params.get('color', 'blue')
    
    # Robot body corners in local frame
    corners = np.array([
        [-LB, -width/2],
        [LF, -width/2],
        [LF, width/2],
        [-LB, width/2]
    ])
    
    # Rotation matrix
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    R = np.array([[cos_yaw, -sin_yaw],
                  [sin_yaw, cos_yaw]])
    
    # Transform corners to world frame
    world_corners = corners @ R.T + np.array([x, y])
    
    # Draw robot body
    rect = plt.Polygon(world_corners, closed=True, 
                      facecolor=color, edgecolor='black', 
                      alpha=alpha, linewidth=2)
    ax.add_patch(rect)
    
    # Draw direction indicator (front center)
    front_x = x + LF * cos_yaw
    front_y = y + LF * sin_yaw
    ax.plot([x, front_x], [y, front_y], 'k-', linewidth=3)
    ax.plot(front_x, front_y, 'ko', markersize=6)

def visualize_static(map_file, config_file=None):
    """Create a static visualization of the heterogeneous fleet"""
    
    # Load configuration
    if config_file:
        load_config(config_file)
    
    # Load map
    with open(map_file, 'r') as f:
        map_data = yaml.safe_load(f)
    
    fig, ax = plt.subplots(figsize=(12, 12))
    
    # Set map dimensions
    dims = map_data['map']['dimensions']
    ax.set_xlim(-2, dims[0] + 2)
    ax.set_ylim(-2, dims[1] + 2)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title('Heterogeneous Fleet - HA-CL-CBS\nStart (solid) and Goal (transparent) Positions', 
                 fontsize=14, fontweight='bold')
    
    # Draw obstacles
    if 'obstacles' in map_data['map']:
        for obs in map_data['map']['obstacles']:
            circle = Circle((obs[0], obs[1]), 1.0, 
                          facecolor='gray', edgecolor='black', alpha=0.5)
            ax.add_patch(circle)
    
    # Draw boundary
    rect = Rectangle((0, 0), dims[0], dims[1], 
                    facecolor='none', edgecolor='red', linewidth=3)
    ax.add_patch(rect)
    
    # Draw agents
    for agent in map_data['agents']:
        name = agent['name']
        robot_type = agent.get('type', 'STANDARD')
        start = agent['start']
        goal = agent['goal']
        
        # Draw start position (solid)
        draw_robot(ax, start[0], start[1], start[2], robot_type, alpha=1.0)
        ax.text(start[0], start[1] - 2, f"{name}\n({robot_type})", 
               ha='center', va='top', fontsize=10, fontweight='bold')
        
        # Draw goal position (transparent)
        draw_robot(ax, goal[0], goal[1], goal[2], robot_type, alpha=0.3)
        ax.text(goal[0], goal[1] - 2, f"Goal {name}", 
               ha='center', va='top', fontsize=9, style='italic')
    
    # Add legend
    legend_elements = []
    for rtype, params in ROBOT_TYPES.items():
        color = params.get('color', 'blue')
        legend_elements.append(
            plt.Rectangle((0, 0), 1, 1, fc=color, 
                        label=f"{rtype} (R={params.get('R_min', 'N/A')}m)")
        )
    ax.legend(handles=legend_elements, loc='upper right', fontsize=10)
    
    plt.tight_layout()
    
    # Save figure
    output_file = map_file.replace('.yaml', '_heterogeneous.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Saved visualization to: {output_file}")
    
    plt.show()

def main():
    parser = argparse.ArgumentParser(
        description='Visualize heterogeneous robot fleet for HA-CL-CBS')
    parser.add_argument('-m', '--map', required=True,
                       help='Map file (YAML)')
    parser.add_argument('-c', '--config', 
                       default='../src/heterogeneous_config.yaml',
                       help='Robot types configuration file (YAML)')
    
    args = parser.parse_args()
    
    visualize_static(args.map, args.config)

if __name__ == '__main__':
    main()
