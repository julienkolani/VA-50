"""
World Model - Unified World Representation

Central repository for all world state:
- Robot poses (filtered by Kalman)
- Occupancy grid (obstacles)
- Arena boundaries
- Coordinate frames

This is the single source of truth for spatial information.
All other modules query the world model.

Does NOT contain game logic (scores, etc.) - only spatial state.
"""

from typing import Dict, List, Tuple
from .occupancy_grid import OccupancyGrid
from .coordinate_frames import TransformManager


class WorldModel:
    """
    Complete spatial world representation.
    
    Manages:
    - Robot state (positions, velocities, orientations)
    - Obstacles (static + dynamic)
    - Coordinate transformations
    - Arena boundaries
    """
    
    def __init__(self, arena_width_m: float, arena_height_m: float, 
                 grid_resolution_m: float = 0.02,
                 robot_radius_m: float = 0.09,
                 inflation_margin_m: float = 0.05):
        """
        Initialize world model.
        
        Args:
            arena_width_m: Arena width from calibration
            arena_height_m: Arena height from calibration
            grid_resolution_m: Grid cell size
            robot_radius_m: Physical robot radius (from config)
            inflation_margin_m: Safety margin for pathfinding (from config)
        """
        self.arena_width = arena_width_m
        self.arena_height = arena_height_m
        self.robot_radius_m = robot_radius_m
        self.inflation_margin_m = inflation_margin_m
        
        # Occupancy grid
        self.grid = OccupancyGrid(arena_width_m, arena_height_m, grid_resolution_m)
        
        # Robot state
        self.robots = {
            4: {  # AI robot
                'pose': (0.0, 0.0, 0.0),  # (x, y, theta)
                'velocity': (0.0, 0.0, 0.0),  # (vx, vy, omega)
                'radius_m': robot_radius_m,
            },
            5: {  # Human robot
                'pose': (0.0, 0.0, 0.0),
                'velocity': (0.0, 0.0, 0.0),
                'radius_m': robot_radius_m,
            }
        }
        
        # Coordinate transforms
        self.transforms = TransformManager()
        
    def update_robot_pose(self, robot_id: int, pose: Tuple[float, float, float]):
        """
        Update robot pose from Kalman filter.
        
        Args:
            robot_id: 4 or 5
            pose: (x, y, theta) in meters/radians
        """
        if robot_id in self.robots:
            self.robots[robot_id]['pose'] = pose
    
    def update_robot_velocity(self, robot_id: int, 
                             velocity: Tuple[float, float, float]):
        """
        Update robot velocity from Kalman filter.
        
        Args:
            robot_id: 4 or 5
            velocity: (vx, vy, omega) in m/s and rad/s
        """
        if robot_id in self.robots:
            self.robots[robot_id]['velocity'] = velocity
    
    def update_occupancy(self):
        """
        Update occupancy grid with current robot positions.
        
        Called each frame after robot poses are updated.
        """
        robot_poses = [self.robots[rid]['pose'] for rid in [4, 5]]
        self.grid.update_dynamic_obstacles(robot_poses, self.robot_radius_m)
    
    def generate_costmap(self):
        """
        Génère la costmap gonflée pour le pathfinding A*.
        
        Appelle après avoir chargé les obstacles statiques.
        Utilise les paramètres robot du config.
        """
        self.grid.inflate_static_obstacles(self.robot_radius_m, self.inflation_margin_m)
        print(f"[WORLD] Costmap générée avec rayon={self.robot_radius_m}m, marge={self.inflation_margin_m}m")
    
    def get_robot_pose(self, robot_id: int) -> Tuple[float, float, float]:
        """Get current robot pose."""
        return self.robots[robot_id]['pose']
    
    def get_robot_velocity(self, robot_id: int) -> Tuple[float, float, float]:
        """Get current robot velocity."""
        return self.robots[robot_id]['velocity']
    
    def is_position_valid(self, x: float, y: float) -> bool:
        """
        Check if a position is within arena and not occupied.
        
        Args:
            x, y: Position in meters
            
        Returns:
            True if position is valid (in bounds and free)
        """
        # Check bounds
        if not (0 <= x <= self.arena_width and 0 <= y <= self.arena_height):
            return False
        
        # Check occupancy
        return not self.grid.is_occupied(x, y)
    
    def get_state_dict(self) -> Dict:
        """
        Export complete world state as dictionary.
        
        Used by AI, game engine, visualization.
        
        Returns:
            dict with all world information
        """
        return {
            'arena_size': (self.arena_width, self.arena_height),
            'robot_4_pose': self.robots[4]['pose'],
            'robot_5_pose': self.robots[5]['pose'],
            'robot_4_velocity': self.robots[4]['velocity'],
            'robot_5_velocity': self.robots[5]['velocity'],
            'occupancy_grid': self.grid,
        }
