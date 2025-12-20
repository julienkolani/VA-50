"""
Debug Draw - Visualisation Debug

Affiche éléments de debug (paths, LOS, grille, etc.).

Logs: [DEBUG_DRAW] prefix
"""

import pygame
import numpy as np
from typing import List, Tuple, Optional
from .colors import *


class DebugDraw:
    """
    Gère affichage éléments debug.
    """
    
    def __init__(self, surface: pygame.Surface, projector_mapping):
        """
        Initialize debug drawer.
        
        Args:
            surface: Surface Pygame
            projector_mapping: ProjectorMapping instance
        """
        self.surface = surface
        self.mapping = projector_mapping
        self.font = pygame.font.SysFont('Courier', 16)
        
    def draw_path(self, waypoints: List[Tuple[float, float]], 
                 color: Tuple = PATH_COLOR):
        """
        Dessine chemin planifié.
        
        Args:
            waypoints: Liste (x, y) en mètres
            color: Couleur ligne
        """
        if len(waypoints) < 2:
            return
        
        # Convertir en pixels
        points_px = [self.mapping.world_to_projector(x, y) 
                    for x, y in waypoints]
        
        # Dessiner lignes
        pygame.draw.lines(self.surface, color, False, points_px, 3)
        
        # Dessiner waypoints
        for px in points_px:
            pygame.draw.circle(self.surface, WAYPOINT_COLOR, px, 5)
            
    def draw_line_of_sight(self,
                          start: Tuple[float, float],
                          end: Tuple[float, float],
                          blocked: bool = False):
        """
        Dessine ligne de vue.
        
        Args:
            start: Point départ (x, y) mètres
            end: Point arrivée (x, y) mètres
            blocked: True si bloquée
        """
        p1 = self.mapping.world_to_projector(*start)
        p2 = self.mapping.world_to_projector(*end)
        
        color = RED if blocked else LOS_LINE_COLOR
        pygame.draw.line(self.surface, color, p1, p2, 2)
        
    def draw_grid(self, grid_resolution: float = 0.5):
        """
        Dessine grille métrique.
        
        Args:
            grid_resolution: Espacement grille (mètres)
        """
        # Lignes verticales
        x = 0.0
        while x <= self.mapping.arena_width_m:
            p1 = self.mapping.world_to_projector(x, 0)
            p2 = self.mapping.world_to_projector(x, self.mapping.arena_height_m)
            pygame.draw.line(self.surface, GRID_LINE_COLOR, p1, p2, 1)
            x += grid_resolution
        
        # Lignes horizontales
        y = 0.0
        while y <= self.mapping.arena_height_m:
            p1 = self.mapping.world_to_projector(0, y)
            p2 = self.mapping.world_to_projector(self.mapping.arena_width_m, y)
            pygame.draw.line(self.surface, GRID_LINE_COLOR, p1, p2, 1)
            y += grid_resolution
            
    def draw_occupancy_grid(self, grid):
        """
        Dessine grille occupation.
        
        Args:
            grid: OccupancyGrid instance
        """
        # Simplification: dessiner cellules occupées
        for row in range(grid.n_rows):
            for col in range(grid.n_cols):
                if grid.grid[row, col] > 0.5:
                    # Convertir cell -> monde -> pixels
                    x_m, y_m = grid.grid_to_world(row, col)
                    px, py = self.mapping.world_to_projector(x_m, y_m)
                    
                    cell_size = self.mapping.scale_length(grid.resolution)
                    rect = pygame.Rect(px, py, cell_size, cell_size)
                    
                    alpha = int(grid.grid[row, col] * 150)
                    s = pygame.Surface((cell_size, cell_size))
                    s.set_alpha(alpha)
                    s.fill(OBSTACLE_COLOR)
                    self.surface.blit(s, rect)
                    
    def draw_robot_info(self, robot_id: int,
                       pose: Tuple[float, float, float],
                       velocity: Optional[Tuple[float, float, float]] = None):
        """
        Affiche infos robot au-dessus.
        
        Args:
            robot_id: ID robot
            pose: (x, y, theta)
            velocity: (vx, vy, omega) optionnel
        """
        x, y, theta = pose
        px, py = self.mapping.world_to_projector(x, y)
        
        # Texte infos
        info_lines = [f"R{robot_id}"]
        info_lines.append(f"({x:.2f}, {y:.2f})")
        
        if velocity:
            vx, vy, omega = velocity
            v_norm = np.sqrt(vx**2 + vy**2)
            info_lines.append(f"v={v_norm:.2f}m/s")
        
        # Dessiner texte
        y_offset = -40
        for line in info_lines:
            text = self.font.render(line, True, BLACK)
            text_rect = text.get_rect(center=(px, py + y_offset))
            
            # Fond
            bg = text_rect.inflate(10, 4)
            s = pygame.Surface((bg.width, bg.height))
            s.set_alpha(180)
            s.fill(WHITE)
            self.surface.blit(s, bg)
            
            self.surface.blit(text, text_rect)
            y_offset += 20
