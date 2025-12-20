"""
Arena Solver - Calcul Dimensions Arène

Déduit les dimensions physiques de l'arène (Lx, Ly)
à partir de la calibration.

Logs: [ARENA_SOLVER] prefix
"""

import numpy as np
from typing import Tuple


class ArenaSolver:
    """
    Calcule dimensions arène depuis calibration.
    """
    
    def __init__(self):
        """Initialize arena solver."""
        self.width_m = None
        self.height_m = None
        
    def solve_from_av_and_scale(self,
                                av_width: float,
                                av_height: float,
                                scale: float) -> Tuple[float, float]:
        """
        Calcule dimensions arène depuis taille AV et échelle.
        
        Args:
            av_width: Largeur en unités AV (typiquement 1.0)
            av_height: Hauteur en unités AV (typiquement 1.0)
            scale: Échelle m/unité_av
            
        Returns:
            (width_m, height_m) dimensions en mètres
            
        Logs:
            [ARENA_SOLVER] Arena dimensions: Lx x Ly meters
        """
        self.width_m = av_width * scale
        self.height_m = av_height * scale
        
        print("[ARENA_SOLVER] Dimensions de l'arène : "
              "{:.2f}m x {:.2f}m".format(self.width_m, self.height_m))
        
        return (self.width_m, self.height_m)
    
    def solve_from_corners(self,
                          corners_world: np.ndarray) -> Tuple[float, float]:
        """
        Calcule dimensions depuis coins arène en coordonnées monde.
        
        Args:
            corners_world: 4 coins en mètres (ordre: BL, BR, TR, TL)
            
        Returns:
            (width_m, height_m)
        """
        # Distance entre coins bas
        width = np.linalg.norm(corners_world[1] - corners_world[0])
        
        # Distance entre coins gauche
        height = np.linalg.norm(corners_world[3] - corners_world[0])
        
        self.width_m = width
        self.height_m = height
        
        print("[ARENA_SOLVER] Dimensions de l'arène depuis coins : "
              "{:.2f}m x {:.2f}m".format(width, height))
        
        return (width, height)
    
    def get_dimensions(self) -> Tuple[float, float]:
        """
        Retourne dimensions calculées.
        
        Returns:
            (width_m, height_m)
            
        Raises:
            ValueError: Si dimensions pas encore calculées
        """
        if self.width_m is None:
            raise ValueError("Dimensions pas encore calculées")
        
        return (self.width_m, self.height_m)
    
    def get_aspect_ratio(self) -> float:
        """Retourne ratio aspect width/height."""
        if self.width_m is None:
            raise ValueError("Dimensions pas encore calculées")
        
        return self.width_m / self.height_m
