"""
Trames de Coordonnées - Gestion des Transformations

Gère toutes les transformations de coordonnées :
- Caméra -> Arène Virtuelle (H_C2AV) - homographie depuis les coins projetés
- Arène Virtuelle -> Monde (mise à l'échelle) - étalonnage métrique
- Caméra -> Monde (H_C2W) - transformation combinée
- Monde -> Pygame (affichage projection)
- Monde -> Projecteur (affichage superposition)

Toutes les transformations sont des homographies 2D ou des transformations affines.

Logs : préfixe [TRANSFORM] pour toutes les opérations de transformation
"""

import numpy as np
import cv2
from typing import Tuple, List


class TransformManager:
    """
    Gère les transformations de trames de coordonnées.
    
    Stocke et applique les homographies entre différents systèmes de coordonnées.
    """
    
    def __init__(self):
        """Initialise le gestionnaire de transformations."""
        self.H_C2AV = None  # Caméra -> Arène Virtuelle
        self.H_AV2W = None  # Arène Virtuelle -> Monde (échelle)
        self.H_C2W = None   # Caméra -> Monde (combiné)
        self.H_W2Proj = None  # Monde -> Projecteur (affichage)
        
        self.scale_m_per_av = None  # Metric scale factor
        
    def set_camera_to_av(self, src_points: np.ndarray, dst_points: np.ndarray):
        """
        Calcule H_C2AV à partir des correspondances de coins.
        
        Args:
            src_points: Tableau 4x2 de coordonnées pixels caméra
            dst_points: Tableau 4x2 de coordonnées arène virtuelle (ex: carré unitaire)
            
        Calcule l'homographie utilisant cv2.findHomography.
        
        Logs:
            [TRANSFORM] H_C2AV calculé à partir de 4 coins
        """
        self.H_C2AV, _ = cv2.findHomography(src_points, dst_points)
        self._update_combined()
        
    def set_av_to_world_scale(self, scale: float):
        """
        Définit l'échelle de l'Arène Virtuelle vers le Monde (mètres).
        
        Args:
            scale: mètres par unité AV
            
        Crée la transformation d'échelle H_AV2W.
        
        Logs:
            [TRANSFORM] Echelle définie : 1.15 m/unité_AV
        """
        self.scale_m_per_av = scale
        self.H_AV2W = np.array([
            [scale, 0, 0],
            [0, scale, 0],
            [0, 0, 1]
        ], dtype=np.float32)
        self._update_combined()
        
    def _update_combined(self):
        """Update H_C2W = H_AV2W @ H_C2AV."""
        if self.H_C2AV is not None and self.H_AV2W is not None:
            self.H_C2W = self.H_AV2W @ self.H_C2AV
            
    def camera_to_world(self, u: float, v: float) -> Tuple[float, float]:
        """
        Transforme pixel caméra en mètres monde.
        
        Args:
            u, v: Coordonnées pixel caméra
            
        Returns:
            (x, y) en mètres
        """
        if self.H_C2W is None:
            raise ValueError("H_C2W non défini, lancez l'étalonnage d'abord")
        
        # Coordonnées homogènes
        p_cam = np.array([u, v, 1.0])
        p_world = self.H_C2W @ p_cam
        
        # Normalisation
        x = p_world[0] / p_world[2]
        y = p_world[1] / p_world[2]
        
        return (x, y)
    
    def world_to_projector(self, x: float, y: float, 
                          arena_width_m: float, arena_height_m: float,
                          proj_width_px: int, proj_height_px: int,
                          margin_px: int = 50) -> Tuple[int, int]:
        """
        Transforme position monde en pixel projecteur.
        
        Args:
            x, y: Position monde en mètres
            arena_width_m, arena_height_m: Taille arène
            proj_width_px, proj_height_px: Résolution projecteur
            margin_px: Marge de sécurité
            
        Returns:
            (px, py) coordonnées pixel projecteur
        """
        # Mise à l'échelle vers projecteur (avec marge)
        draw_width = proj_width_px - 2 * margin_px
        draw_height = proj_height_px - 2 * margin_px
        
        scale_x = draw_width / arena_width_m
        scale_y = draw_height / arena_height_m
        scale = min(scale_x, scale_y)  # Conserve le ratio d'aspect
        
        px = margin_px + int(x * scale)
        py = margin_px + int((arena_height_m - y) * scale)  # Inverse Y (origine pygame en haut à gauche)
        
        return (px, py)
    
    def batch_camera_to_world(self, points_cam: np.ndarray) -> np.ndarray:
        """
        Transforme plusieurs points caméra vers monde.
        
        Args:
            points_cam: Tableau Nx2 de coordonnées caméra
            
        Returns:
            Tableau Nx2 de coordonnées monde
        """
        if self.H_C2W is None:
            raise ValueError("H_C2W non défini")
        
        # Ajoute coordonnée homogène
        ones = np.ones((points_cam.shape[0], 1))
        points_h = np.hstack([points_cam, ones])
        
        # Transform
        points_world_h = (self.H_C2W @ points_h.T).T
        
        # Normalisation
        points_world = points_world_h[:, :2] / points_world_h[:, 2:3]
        
        return points_world
