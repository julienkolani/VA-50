"""
Occupancy Grid - 2D Spatial Representation

Represents the arena as a 2D grid with metric resolution:
- Cell values: 0 = free, 1 = occupied, 0-1 = partial
- Resolution: typically 2cm (0.02m) per cell
- Dimensions: derived from calibration

The grid stores:
- Static obstacles (from calibration)
- Dynamic obstacles (robots, updated each frame)
- Inflated obstacles (safety margins)

Coordinate system: meters, origin at arena bottom-left.

Logs: [GRID] prefix for all grid operations
"""

import numpy as np
from typing import Tuple, List


class OccupancyGrid:
    """
    2D occupancy grid for spatial representation.
    
    Provides efficient collision checking and spatial queries.
    """
    
    def __init__(self, width_m: float, height_m: float, resolution_m: float = 0.02):
        """
        Initialize occupancy grid.
        
        Args:
            width_m: Arena width in meters
            height_m: Arena height in meters  
            resolution_m: Grid cell size in meters (default 2cm)
            
        The grid will have dimensions:
            n_cols = ceil(width_m / resolution_m)
            n_rows = ceil(height_m / resolution_m)
            
        Logs:
            [GRID] Created grid: 2.85m x 1.90m @ 0.02m -> 143 x 95 cells
        """
        self.width_m = width_m
        self.height_m = height_m
        self.resolution = resolution_m
        
        self.n_cols = int(np.ceil(width_m / resolution_m))
        self.n_rows = int(np.ceil(height_m / resolution_m))
        
        # Grid data: 0 = free, 1 = occupied
        self.grid = np.zeros((self.n_rows, self.n_cols), dtype=np.float32)
        
        # Static obstacles (from calibration, never change)
        self.static_grid = np.zeros((self.n_rows, self.n_cols), dtype=np.float32)
        
    def world_to_grid(self, x_m: float, y_m: float) -> Tuple[int, int]:
        """
        Convert world coordinates to grid cell.
        
        Args:
            x_m: X position in meters
            y_m: Y position in meters
            
        Returns:
            (row, col) grid cell indices
        """
        col = int(x_m / self.resolution)
        row = int(y_m / self.resolution)
        return (row, col)
    
    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """
        Convert grid cell to world coordinates (cell center).
        
        Args:
            row: Grid row
            col: Grid column
            
        Returns:
            (x_m, y_m) in meters
        """
        x_m = (col + 0.5) * self.resolution
        y_m = (row + 0.5) * self.resolution
        return (x_m, y_m)
    
    def is_occupied(self, x_m: float, y_m: float, threshold: float = 0.5) -> bool:
        """
        Check if a world position is occupied.
        
        Args:
            x_m: X position in meters
            y_m: Y position in meters
            threshold: Occupancy threshold (0-1)
            
        Returns:
            True if occupied
        """
        row, col = self.world_to_grid(x_m, y_m)
        
        if not self._is_valid_cell(row, col):
            return True  # Out of bounds = occupied
        
        return self.grid[row, col] >= threshold
    
    def get_value(self, x_m: float, y_m: float) -> float:
        """
        Get occupancy value at a world position.
        
        Args:
            x_m: X position in meters
            y_m: Y position in meters
            
        Returns:
            Occupancy value 0-100 (0=free, 100=occupied)
        """
        row, col = self.world_to_grid(x_m, y_m)
        
        if not self._is_valid_cell(row, col):
            return 100  # Out of bounds = occupied
        
        return self.grid[row, col] * 100
    
    def set_static_obstacles(self, obstacle_cells: List[Tuple[int, int]]):
        """
        Set static obstacles from calibration.
        
        Args:
            obstacle_cells: List of (row, col) occupied cells
            
        Logs:
            [GRID] Static obstacles set: N cells
        """
        for row, col in obstacle_cells:
            if self._is_valid_cell(row, col):
                self.static_grid[row, col] = 1.0
        
        # Par défaut, inflated_grid = static_grid (pas d'inflation)
        self.inflated_grid = self.static_grid.copy()
        self.grid = self.inflated_grid.copy()
        
        print(f"[GRID] Static obstacles set: {len(obstacle_cells)} cells")
    
    def inflate_static_obstacles(self, robot_radius_m: float, safety_margin_m: float = 0.0):
        """
        Génère une Costmap en gonflant les obstacles avec cv2.dilate (RAPIDE).
        
        Utilise OpenCV pour un calcul instantané même sur Raspberry Pi.
        Crée une zone tampon autour de chaque obstacle basée sur la taille physique du robot.
        
        Args:
            robot_radius_m: Rayon physique du robot (ex: 0.09m pour Turtlebot)
            safety_margin_m: Marge de sécurité supplémentaire (ex: 0.05m)
            
        Logs:
            [GRID] Inflation calculée: rayon + marge = total => kernel size
        """
        import cv2
        
        # 1. Calcul du rayon total à gonfler
        total_inflation_m = robot_radius_m + safety_margin_m
        
        # 2. Conversion Mètres -> Cellules (dynamique selon résolution)
        radius_cells = int(np.ceil(total_inflation_m / self.resolution))
        
        # 3. Calcul du Kernel pour OpenCV (doit être impair: 3x3, 5x5, 7x7...)
        kernel_size = (radius_cells * 2) + 1
        
        print(f"[GRID] Inflation calculée:")
        print(f"       Robot: {robot_radius_m}m + Marge: {safety_margin_m}m = {total_inflation_m}m")
        print(f"       Cellules: {total_inflation_m:.3f}m / {self.resolution}m = {radius_cells} cells")
        print(f"       Kernel OpenCV: {kernel_size}x{kernel_size}")
        
        # 4. Création du Kernel Circulaire (forme du robot)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        
        # 5. Application de la dilatation (ULTRA RAPIDE vs boucles Python)
        static_u8 = (self.static_grid * 255).astype(np.uint8)
        inflated_u8 = cv2.dilate(static_u8, kernel)
        
        # 6. Sauvegarde dans costmap (pour A*) et inflated_grid
        self.costmap = (inflated_u8 > 127).astype(np.float32)
        self.inflated_grid = self.costmap.copy()
        
        # Appliquer à la grille courante
        self.grid = self.inflated_grid.copy()
    
    def update_dynamic_obstacles(self, robot_poses: List[Tuple[float, float, float]], 
                                 robot_radius_m: float):
        """
        Update grid with current robot positions.
        
        Args:
            robot_poses: List of (x, y, theta) for each robot
            robot_radius_m: Robot radius in meters
            
        Algorithm:
            1. Reset grid to inflated static obstacles
            2. For each robot, mark cells in radius as occupied
        """
        # Reset to inflated static (pas static_grid brut)
        self.grid = self.inflated_grid.copy() if hasattr(self, 'inflated_grid') else self.static_grid.copy()
        
        # Add robot footprints
        radius_cells = int(np.ceil(robot_radius_m / self.resolution))
        
        for x, y, _ in robot_poses:
            center_row, center_col = self.world_to_grid(x, y)
            
            # Mark circle of cells
            for dr in range(-radius_cells, radius_cells + 1):
                for dc in range(-radius_cells, radius_cells + 1):
                    if dr**2 + dc**2 <= radius_cells**2:
                        r, c = center_row + dr, center_col + dc
                        if self._is_valid_cell(r, c):
                            self.grid[r, c] = 1.0
    
    def _is_valid_cell(self, row: int, col: int) -> bool:
        """Check if cell is within grid bounds."""
        return 0 <= row < self.n_rows and 0 <= col < self.n_cols
    
    def get_costmap(self):
        """
        Return current costmap for planning.
        
        Returns:
            numpy array (n_rows x n_cols) with costs 0-100
        """
        return (self.grid * 100).astype(np.uint8)

