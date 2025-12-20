"""
Projector Mapping - Transformation Monde -> Projecteur

Gère la conversion des coordonnées monde (mètres)
vers pixels projecteur pour affichage Pygame.

Logs: [PROJ_MAP] prefix
"""

import numpy as np
from typing import Tuple


class ProjectorMapping:
    """
    Mapping Monde -> Pixels Projecteur.
    """
    
    def __init__(self,
                 projector_width: int = 1024,
                 projector_height: int = 768,
                 margin: int = 50):
        """
        Initialize projector mapping.
        
        Args:
            projector_width: Résolution projecteur largeur
            projector_height: Résolution projecteur hauteur
            margin: Marge sécurité (pixels)
        """
        self.proj_w = projector_width
        self.proj_h = projector_height
        self.margin = margin
        
        self.draw_w = projector_width - 2 * margin
        self.draw_h = projector_height - 2 * margin
        
        # Paramètres monde (à définir après calibration)
        self.arena_width_m = None
        self.arena_height_m = None
        self.scale = None
        
    def set_arena_dimensions(self, width_m: float, height_m: float):
        """
        Définit dimensions arène et calcule échelle d'affichage.
        
        Args:
            width_m: Largeur arène en mètres
            height_m: Hauteur arène en mètres
            
        Logs:
            [PROJ_MAP] Arena set: WxH m, scale: S px/m
        """
        self.arena_width_m = width_m
        self.arena_height_m = height_m
        
        # Calculer échelle (maintenir aspect ratio)
        scale_x = self.draw_w / width_m
        scale_y = self.draw_h / height_m
        self.scale = min(scale_x, scale_y)
        
        print("[PROJ_MAP] Arène définie : {:.2f}x{:.2f}m, "
              "échelle : {:.1f} px/m".format(width_m, height_m, self.scale))
        
    def world_to_projector(self, x_m: float, y_m: float) -> Tuple[int, int]:
        """
        Convertit coordonnées monde -> pixels projecteur.
        
        Args:
            x_m, y_m: Position en mètres
            
        Returns:
            (px, py) position en pixels projecteur
        """
        if self.scale is None:
            raise ValueError("Appeler set_arena_dimensions d'abord")
        
        # Conversion avec flip Y (pygame origin top-left)
        px = self.margin + int(x_m * self.scale)
        py = self.margin + int((self.arena_height_m - y_m) * self.scale)
        
        return (px, py)
    
    def projector_to_world(self, px: int, py: int) -> Tuple[float, float]:
        """
        Convertit pixels projecteur -> coordonnées monde.
        
        Args:
            px, py: Position en pixels
            
        Returns:
            (x_m, y_m) position en mètres
        """
        if self.scale is None:
            raise ValueError("Appeler set_arena_dimensions d'abord")
        
        x_m = (px - self.margin) / self.scale
        y_m = self.arena_height_m - (py - self.margin) / self.scale
        
        return (x_m, y_m)
    
    def scale_length(self, length_m: float) -> int:
        """
        Convertit longueur mètres -> pixels.
        
        Args:
            length_m: Longueur en mètres
            
        Returns:
            Longueur en pixels
        """
        if self.scale is None:
            raise ValueError("Appeler set_arena_dimensions d'abord")
        
        return int(length_m * self.scale)
    
    def get_safe_zone_rect(self) -> Tuple[int, int, int, int]:
        """
        Retourne rectangle zone sécurité.
        
        Returns:
            (x, y, width, height) en pixels
        """
        return (self.margin, self.margin, self.draw_w, self.draw_h)
