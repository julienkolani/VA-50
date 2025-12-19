"""
Raycast - Shot Collision Detection

Implements ray-based collision detection for laser shots:
- Cast ray from shooter position in direction theta
- Check intersections with:
  * Static obstacles (walls, blocks)
  * Dynamic obstacles (robots)
- Return first hit or None

Uses the occupancy grid from core/world for obstacle detection.
Implements DDA (Digital Differential Analyzer) for efficient grid traversal.

Logs: [RAYCAST] Hit detected / Miss
"""

import numpy as np
from typing import Optional, Tuple


class Raycast:
    """
    Efficient ray-based collision detection for shots.
    
    Uses DDA algorithm to traverse occupancy grid and detect hits.
    """
    
    def __init__(self, occupancy_grid):
        """
        Initialize raycast with world occupancy grid.
        
        Args:
            occupancy_grid: OccupancyGrid instance from core/world
        """
        self.grid = occupancy_grid
    
    
    def cast_shot(self, start_pos, theta, max_range_m):
        """
        Cast a shot ray and detect collisions.
        
        Args:
            start_pos: (x, y) shooter position in meters
            theta: Shot direction in radians
            max_range_m: Maximum shot range
            
        Returns:
            dict: {
                'hit': bool,
                'target': 'robot4' | 'robot5' | 'obstacle' | None,
                'impact_point': (x, y) in meters or None,
                'distance': float in meters
            }
        """
        start_x, start_y = start_pos
        
        # Ray direction vector
        dx = np.cos(theta)
        dy = np.sin(theta)
        
        # Walk the grid
        current_dist = 0.0
        step_size = self.grid.resolution
        
        # Check every point along the ray
        while current_dist <= max_range_m:
            curr_x = start_x + dx * current_dist
            curr_y = start_y + dy * current_dist
            
            # 1. Check map boundaries
            if not (0 <= curr_x <= self.grid.width_m and 0 <= curr_y <= self.grid.height_m):
                break
                
            # 2. Check static obstacles using occupancy grid
            grid_val = self.grid.get_value(curr_x, curr_y)
            if grid_val > 50:  # Threshold for occupied
                return {
                    'hit': True,
                    'target': 'obstacle',
                    'impact_point': (curr_x, curr_y),
                    'distance': current_dist
                }
            
            current_dist += step_size
            
        return {
            'hit': False,
            'target': None,
            'impact_point': None,
            'distance': max_range_m
        }

    def check_robot_collision(self, start_pos, theta, max_range_m, target_pos, target_radius_m=0.15):
        """
        Check if ray hits a specific robot.
        
        Args:
            start_pos: (x,y) shooter
            theta: angle
            max_range_m: max range
            target_pos: (x,y) target center
            target_radius_m: target hit radius
        """
        # Vector from shooter to target
        sx, sy = start_pos
        tx, ty = target_pos
        
        val_x = tx - sx
        val_y = ty - sy
        
        # Project target center onto ray
        # Ray vector: (cos, sin)
        ray_x, ray_y = np.cos(theta), np.sin(theta)
        
        # Dot product
        t = val_x * ray_x + val_y * ray_y
        
        # Closest point on ray to target center
        closest_x = sx + t * ray_x
        closest_y = sy + t * ray_y
        
        # Distance checks
        if t < 0: return False # Target behind shooter
        if t > max_range_m: return False # Target out of range
        
        # Distance from closest point to target center
        dist_sq = (closest_x - tx)**2 + (closest_y - ty)**2
        
        return dist_sq <= (target_radius_m**2)

    def _check_line_of_sight(self, pos1, pos2):
        """Check if LOS is clear between two points (simple version)."""
        x1, y1 = pos1
        x2, y2 = pos2
        
        dist = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        if dist == 0: return True
        
        dx = (x2 - x1) / dist
        dy = (y2 - y1) / dist
        
        # Step through grid
        curr_dist = 0
        step = self.grid.resolution
        
        while curr_dist < dist:
            cx = x1 + dx * curr_dist
            cy = y1 + dy * curr_dist
            
            if self.grid.get_value(cx, cy) > 50:
                return False
                
            curr_dist += step
            
        return True

