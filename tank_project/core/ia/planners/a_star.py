"""
A* Path Planning Algorithm

Implements A* pathfinding on the occupancy grid:
- Finds shortest collision-free path
- Uses configurable heuristics
- Handles dynamic obstacles (robots)
- Returns waypoint list in meters

The planner works on the inflated costmap from core/world.

Logs: [ASTAR] Path found: N waypoints, length: M meters
"""

import numpy as np
import heapq
from typing import List, Tuple, Optional
from .heuristics import euclidean_distance, manhattan_distance, diagonal_distance


class AStarPlanner:
    """
    A* pathfinding on 2D occupancy grid.
    
    Finds optimal path from start to goal avoiding obstacles.
    """
    
    def __init__(self, occupancy_grid, heuristic='euclidean'):
        """
        Initialize A* planner.
        
        Args:
            occupancy_grid: OccupancyGrid from core/world
            heuristic: 'euclidean', 'manhattan', or 'diagonal'
        """
        self.grid = occupancy_grid
        self.heuristic_name = heuristic
        
    def _heuristic(self, cell1, cell2):
        """Calculate heuristic cost."""
        if self.heuristic_name == 'manhattan':
            return manhattan_distance(cell1, cell2)
        elif self.heuristic_name == 'diagonal':
            return diagonal_distance(cell1, cell2)
        else:
            return euclidean_distance(cell1, cell2)

    
    def plan(self, start_m: Tuple[float, float], 
             goal_m: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        Find path from start to goal.
        """
        start_cell = self.grid.world_to_grid(*start_m)
        goal_cell = self.grid.world_to_grid(*goal_m)
        
        print(f"[ASTAR] Planning from {start_m} to {goal_m}")
        print(f"[ASTAR]   Start cell: {start_cell}, Goal cell: {goal_cell}")
        print(f"[ASTAR]   Grid size: {self.grid.grid.shape}")
        
        # Check bounds
        if not self.grid._is_valid_cell(*start_cell):
            print(f"[ASTAR]   ERROR: Start cell {start_cell} is OUT OF BOUNDS")
            return None
        if not self.grid._is_valid_cell(*goal_cell):
            print(f"[ASTAR]   ERROR: Goal cell {goal_cell} is OUT OF BOUNDS")
            return None
        
        # Note: We don't check if start or goal is occupied
        # Start: robot is already there
        # Goal: enemy robot is there - we WANT to go there!
            
        # Initialize
        open_set = []
        heapq.heappush(open_set, (0, start_cell))
        
        came_from = {}
        g_score = {start_cell: 0}
        f_score = {start_cell: self._heuristic(start_cell, goal_cell)}
        
        closed_set = set()
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current == goal_cell:
                path_cells = self._reconstruct_path(came_from, current)
                simplified = self._simplify_path(path_cells)
                result = [self.grid.grid_to_world(r, c) for r, c in simplified]
                print(f"[ASTAR]   SUCCESS: Path found with {len(result)} waypoints")
                return result
            
            closed_set.add(current)
            
            for neighbor, cost in self._get_neighbors(current, start_cell):
                if neighbor in closed_set:
                    continue
                    
                tentative_g = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self._heuristic(neighbor, goal_cell)
                    f_score[neighbor] = f
                    heapq.heappush(open_set, (f, neighbor))
        
        print(f"[ASTAR]   FAILED: No path found (explored {len(closed_set)} cells)")
        return None
        
    def _get_neighbors(self, cell, start_cell=None, ignore_radius=5):
        """Get valid neighbor cells (8-connected).
        
        Uses COSTMAP (inflated) for collision checking, not raw grid.
        If start_cell is provided and we're within ignore_radius of it,
        we ignore obstacles (to escape from robot's own footprint).
        """
        row, col = cell
        neighbors = []
        
        # Use costmap if available (inflated obstacles), fallback to grid
        collision_grid = getattr(self.grid, 'costmap', self.grid.grid)
        
        for dr in [-1, 0, 1]:
            for dc in [-1, 0, 1]:
                if dr == 0 and dc == 0:
                    continue
                    
                r, c = row + dr, col + dc
                
                # Check validity
                if self.grid._is_valid_cell(r, c):
                    # If near start, ignore obstacles (robot's own footprint)
                    if start_cell is not None:
                        dist_to_start = abs(r - start_cell[0]) + abs(c - start_cell[1])
                        if dist_to_start <= ignore_radius:
                            cost = 1.414 if (dr != 0 and dc != 0) else 1.0
                            neighbors.append(((r, c), cost))
                            continue
                    
                    # Check occupancy on COSTMAP (inflated, safety margins)
                    if collision_grid[r, c] < 0.5:
                        cost = 1.414 if (dr != 0 and dc != 0) else 1.0
                        neighbors.append(((r, c), cost))
                        
        return neighbors
    
    def _reconstruct_path(self, came_from, current):
        """Reconstruct path from goal to start."""
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1] # Reverse
    
    def _simplify_path(self, path_cells):
        """Simple path smoothing (skip minimal steps)."""
        if len(path_cells) <= 2:
            return path_cells
            
        simplified = [path_cells[0]]
        for i in range(1, len(path_cells)-1):
            # Keep every Nth point or check Line of Sight (expensive)
            # For now, just return all points to be safe for trajectory follower
            simplified.append(path_cells[i])
            
        simplified.append(path_cells[-1])
        return simplified
