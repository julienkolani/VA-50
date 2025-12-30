"""
Module Tactique IA

Gère l'analyse de l'environnement pour prendre des décisions stratégiques
comme la fuite, le contournement ou le choix de position de tir.
"""

import numpy as np

class TacticalAnalyzer:
    """
    Analyseur tactique pour le robot Tank.
    """
    def __init__(self, world_model):
        """
        Args:
            world_model: Instance de core.world.world_model.WorldModel
        """
        self.world = world_model

    def find_safest_zone(self, enemy_pose, segments_x=5, segments_y=4):
        """
        Trouve la zone la plus éloignée de l'ennemi dans l'arène.
        
        Args:
            enemy_pose: (x, y, theta) de l'ennemi
            segments_x: Nombre de divisions en X
            segments_y: Nombre de divisions en Y
            
        Returns:
            (x, y) de la zone la plus sûre ou None si rien trouvé
        """
        if enemy_pose is None:
            return None
            
        max_dist = -1
        best_zone = None
        
        # Dimensions des secteurs
        sector_w = self.world.arena_width / segments_x
        sector_h = self.world.arena_height / segments_y
        
        for ix in range(segments_x):
            for iy in range(segments_y):
                # Centre du secteur
                cx = (ix + 0.5) * sector_w
                cy = (iy + 0.5) * sector_h
                
                # Vérifie si le point est accessible (pas dans un mur)
                if self.world.is_position_valid(cx, cy):
                    dist = np.hypot(cx - enemy_pose[0], cy - enemy_pose[1])
                    
                    if dist > max_dist:
                        max_dist = dist
                        best_zone = (cx, cy)
                        
        return best_zone
