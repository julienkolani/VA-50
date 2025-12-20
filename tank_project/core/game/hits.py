"""
Touches - Gestion des Scores et Impacts

Gère la détection des touches et le score :
- Valide les touches (cible à portée, non obstruée)
- Enregistre les touches pour les deux robots
- Calcule les deltas de score
- Émet des événements de touche pour la visualisation

Fonctionne avec raycast.py pour la détection de collision.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class HitEvent:
    """
    Représente un événement de touche unique.
    
    Utilisé pour la visualisation (effet flash, son, mise à jour score).
    """
    shooter_id: int  # 4 (AI) or 5 (Human)
    target_id: int   # 4 or 5
    impact_point: tuple  # (x, y) in meters
    timestamp: float
    damage: int = 1  # Future: variable damage
    

class HitManager:
    """
    Gère la validation des touches et le score.
    
    Collabore avec Raycast pour déterminer les touches valides.
    """
    
    def __init__(self, raycast):
        """
        Initialise le gestionnaire de touches.
        
        Args:
            raycast: Instance Raycast pour la détection de collision
        """
        self.raycast = raycast
        self.hit_history = []  # List of HitEvent
        
    
    def process_shot(self, shooter_id, shooter_pose, target_pose, current_time):
        """
        Traite une tentative de tir et détermine si elle touche.
        
        Args:
            shooter_id: 4 (IA) ou 5 (Humain)
            shooter_pose: (x, y, theta) du tireur
            target_pose: (x, y, theta) de la cible
            current_time: Temps de jeu actuel
            
        Returns:
            HitEvent si touché, None si manqué
        """
        x, y, theta = shooter_pose
        target_id = 5 if shooter_id == 4 else 4
        
        target_x, target_y, _ = target_pose
        
        # Portée max codée en dur pour l'instant (devrait venir des règles)
        MAX_RANGE = 5.0 
        
        # 1. Vérifie d'abord les obstacles
        # On ne se soucie que des obstacles plus proches que la cible
        dist_to_target = ((target_x - x)**2 + (target_y - y)**2)**0.5
        
        obstacle_hit = self.raycast.cast_shot((x, y), theta, dist_to_target)
        if obstacle_hit['hit'] and obstacle_hit['target'] == 'obstacle':
            # Obstacle touché avant la cible
            print("[HIT] Tir Robot {} bloqué par obstacle à {:.2f}m".format(shooter_id, obstacle_hit['distance']))
            return None
            
        # 2. Vérifie la collision avec le robot
        is_hit = self.raycast.check_robot_collision(
            (x, y), theta, MAX_RANGE, (target_x, target_y)
        )
        
        if is_hit:
            print("[HIT] Robot {} TOUCHE Robot {} !".format(shooter_id, target_id))
            
            # Calcule le point d'impact (approximatif)
            impact_point = (target_x, target_y) 
            
            event = HitEvent(
                shooter_id=shooter_id,
                target_id=target_id,
                impact_point=impact_point,
                timestamp=current_time
            )
            self.hit_history.append(event)
            return event
            
        return None
    
    def get_score_summary(self):
        """
        Calcule le score actuel à partir de l'historique des touches.
        
        Returns:
            dict: {
                'robot_4_hits': int,  # Touches marquées par IA
                'robot_5_hits': int,  # Touches marquées par Humain
            }
        """
        r4_hits = sum(1 for h in self.hit_history if h.shooter_id == 4)
        r5_hits = sum(1 for h in self.hit_history if h.shooter_id == 5)
        
        return {
            'robot_4_hits_inflicted': r4_hits,
            'robot_5_hits_inflicted': r5_hits,
            'robot_4_hits_received': r5_hits,
            'robot_5_hits_received': r4_hits
        }
    
    def clear_history(self):
        """Efface l'historique des touches (utilisé au démarrage d'un nouveau match)."""
        self.hit_history = []
