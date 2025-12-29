"""
Raycast - Détection de Collision de Tir

Implémente la détection de collision par lancer de rayons pour les tirs laser :
- Lance un rayon depuis la position du tireur dans la direction theta
- Vérifie les intersections avec :
  * Obstacles statiques (murs, blocs)
  * Obstacles dynamiques (robots)
- Retourne le premier impact ou None

Utilise la grille d'occupation de core/world pour la détection d'obstacles.
Implémente DDA (Digital Differential Analyzer) pour une traversée de grille efficace.

Logs : [RAYCAST] Impact détecté / Manqué
"""

import numpy as np
from typing import Optional, Tuple


class Raycast:
    """
    Détection de collision efficace basée sur les rayons pour les tirs.
    
    Utilise l'algorithme DDA pour traverser la grille d'occupation et détecter les impacts.
    """
    
    def __init__(self, occupancy_grid):
        """
        Initialise le raycast avec la grille d'occupation du monde.
        
        Args:
            occupancy_grid: Instance OccupancyGrid de core/world
        """
        self.grid = occupancy_grid
    
    
    def cast_shot(self, start_pos, theta, max_range_m):
        """
        Lance un rayon de tir et détecte les collisions.
        
        Args:
            start_pos: (x, y) position du tireur en mètres
            theta: Direction du tir en radians
            max_range_m: Portée maximale du tir
            
        Returns:
            dict: {
                'hit': bool,
                'target': 'robot4' | 'robot5' | 'obstacle' | None,
                'impact_point': (x, y) en mètres ou None,
                'distance': float en mètres
            }
        """
        start_x, start_y = start_pos
        
        # Vecteur direction du rayon
        dx = np.cos(theta)
        dy = np.sin(theta)
        
        # Parcours de grille
        # FIX: Commencer 20cm devant pour sortir du robot
        current_dist = 0.20
        step_size = self.grid.resolution
        
        # Vérifie chaque point le long du rayon
        while current_dist <= max_range_m:
            curr_x = start_x + dx * current_dist
            curr_y = start_y + dy * current_dist
            
            # 1. Vérifie les limites de la carte
            if not (0 <= curr_x <= self.grid.width_m and 0 <= curr_y <= self.grid.height_m):
                break
                
            # 2. Vérifie les obstacles statiques utilisant la grille d'occupation
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
        Vérifie si le rayon touche un robot spécifique.
        
        Args:
            start_pos: (x,y) tireur
            theta: angle
            max_range_m: portée max
            target_pos: (x,y) centre cible
            target_radius_m: rayon de touche cible
        """
        # Vecteur du tireur à la cible
        sx, sy = start_pos
        tx, ty = target_pos
        
        val_x = tx - sx
        val_y = ty - sy
        
        # Projette le centre de la cible sur le rayon
        # Vecteur rayon : (cos, sin)
        ray_x, ray_y = np.cos(theta), np.sin(theta)
        
        # Produit scalaire
        t = val_x * ray_x + val_y * ray_y
        
        # Point le plus proche sur le rayon vers le centre de la cible
        closest_x = sx + t * ray_x
        closest_y = sy + t * ray_y
        
        # Vérification des distances
        if t < 0: return False # Cible derrière le tireur
        if t > max_range_m: return False # Cible hors de portée
        
        # Distance du point le plus proche au centre de la cible
        dist_sq = (closest_x - tx)**2 + (closest_y - ty)**2
        
        return dist_sq <= (target_radius_m**2)

    def _check_line_of_sight(self, pos1, pos2):
        """Vérifie si la ligne de vue (LOS) est dégagée entre deux points (version simple)."""
        x1, y1 = pos1
        x2, y2 = pos2
        
        dist = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        if dist == 0: return True
        
        dx = (x2 - x1) / dist
        dy = (y2 - y1) / dist
        
        # Pas à pas dans la grille
        curr_dist = 0
        step = self.grid.resolution
        
        while curr_dist < dist:
            cx = x1 + dx * curr_dist
            cy = y1 + dy * curr_dist
            
            if self.grid.get_value(cx, cy) > 50:
                return False
                
            curr_dist += step
            
        return True

