"""
Inflation - Gonflement du Coût des Obstacles

Gonfle les obstacles dans la costmap pour une planification de chemin sûre :
- Ajoute une marge de sécurité autour des obstacles
- Crée un gradient pour une planification plus douce
- Prend en compte la taille du robot

Utilise la transformée de distance pour un calcul efficace.

Logs : [INFLATION] Gonflé avec rayon : Xm -> Y cellules
"""

import numpy as np
from scipy.ndimage import distance_transform_edt


class CostmapInflation:
    """
    Gonfle les obstacles dans la costmap pour une planification avec a start
    
    Crée un gradient de coût autour des obstacles
    """
    
    def __init__(self, inflation_radius_m: float, resolution_m: float):
        """
        Initialise l'inflation.
        
        Args:
            inflation_radius_m: Jusqu'où gonfler en mètres
            resolution_m: Résolution de la grille
        """
        self.inflation_radius_m = inflation_radius_m
        self.resolution = resolution_m
        self.inflation_cells = int(inflation_radius_m / resolution_m)
        
    def inflate(self, binary_grid: np.ndarray) -> np.ndarray:
        """
        Gonfle les obstacles dans la grille.
        
        Args:
            binary_grid: Grille avec 0 = libre, 1 = occupé
            
        Returns:
            Costmap gonflée avec gradient (0-1 float)
            
        Algorithme :
            1. Calcule la transformée de distance (distance à l'obstacle le plus proche)
            2. Convertit les distances en coûts :
               - d = 0 : coût = 1.0 (occupé)
               - d < inflation_radius : coût = 1.0 - (d / rayon)
               - d >= inflation_radius : coût = 0.0 (libre)
               
        Logs:
            [INFLATION] Grille gonflée avec rayon : 0.24m (12 cellules)
        """
        # Transformée de distance : chaque cellule = distance à l'obstacle le plus proche
        distances = distance_transform_edt(1 - binary_grid) * self.resolution
        
        # Convertit en coûts
        costmap = np.zeros_like(distances, dtype=np.float32)
        
        # Cellules occupées
        costmap[binary_grid == 1] = 1.0
        
        # Région gonflée
        inflation_mask = (distances > 0) & (distances < self.inflation_radius_m)
        costmap[inflation_mask] = 1.0 - (distances[inflation_mask] / self.inflation_radius_m)
        
        return costmap
    
    def inflate_discrete(self, binary_grid: np.ndarray, 
                        lethal: int = 100, inscribed: int = 99) -> np.ndarray:
        """
        Gonfle avec des valeurs de coût discrètes (style costmap ROS).
        
        Args:
            binary_grid: Grille d'occupation binaire
            lethal: Coût pour les cellules occupées (défaut 100)
            inscribed: Coût pour les cellules dans le rayon d'inflation
            
        Returns:
            Costmap avec valeurs [0, inscribed, lethal]
        """
        distances = distance_transform_edt(1 - binary_grid)
        
        costmap = np.zeros_like(distances, dtype=np.uint8)
        
        # Obstacles létaux
        costmap[binary_grid == 1] = lethal
        
        # Région inscrite
        inflation_mask = (distances > 0) & (distances <= self.inflation_cells)
        costmap[inflation_mask] = inscribed
        
        return costmap
