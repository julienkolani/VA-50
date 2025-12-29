"""
Suiveur de Trajectoire - Contrôleur Pure Pursuit Optimisé

Implémente le suivi de trajectoire par courbure géométrique :
- Sélection du point lookahead sur le chemin
- Transformation monde → repère robot  
- Calcul de courbure : κ = 2*y_r / L_d²
- Commande angulaire : ω = v * κ
- Alignement sur LOOKAHEAD (pas sur goal final)

Prend un chemin (liste de waypoints) et la pose actuelle, sort (v, ω).
"""

import math
import numpy as np
from typing import Tuple, List


class TrajectoryFollower:
    """
    Contrôleur de suivi de trajectoire pour la navigation par points.
    
    Utilise l'algorithme Pure Pursuit avec alignement sur lookahead local.
    """
    
    def __init__(self, config):
        # État interne pour hystérésis d'alignement
        self._in_alignment_mode = False
        
        # Paramètres
        self.lookahead_distance = config.get('lookahead_distance_m', config.get('lookahead_distance', 0.15))
        self.k_v = config.get('k_velocity', config.get('k_v', 0.6))
        
        # Limites de vitesse
        max_lin = config.get('max_linear_mps', config.get('max_linear_vel', 0.22))
        max_ang = config.get('max_angular_radps', config.get('max_angular_vel', 1.2))
        
        self.max_linear_vel = float(max_lin)
        self.max_angular_vel = float(max_ang)
        
        # Seuil waypoint atteint
        self.waypoint_reached_threshold = config.get('waypoint_threshold_m', config.get('waypoint_threshold', 0.07))
    
    def compute_control(self, current_pose: Tuple[float, float, float], waypoints: List[Tuple[float, float]]) -> Tuple[float, float]:
        """
        Calcule les commandes de contrôle pour suivre les waypoints.
        
        Machine d'état simplifiée :
        1. Trouver le point lookahead
        2. Aligner sur ce point (si > 45°)
        3. Approche finale (si < 2 waypoints restants)
        4. Pure Pursuit sinon
        
        Args:
            current_pose: (x, y, theta) pose actuelle du robot
            waypoints: Liste de waypoints (x, y) en mètres
            
        Returns:
            (v, omega): vitesses linéaire et angulaire
        """
        if not waypoints:
            return (0.0, 0.0)
        
        x, y, theta = current_pose
        
        # 1. Trouver le point à viser (Lookahead)
        target_wp = self._get_lookahead_point(current_pose, waypoints)
        dx = target_wp[0] - x
        dy = target_wp[1] - y
        dist_to_target = math.sqrt(dx**2 + dy**2)
        
        # 2. Alpha calculé sur la CIBLE LOCALE (lookahead), pas sur le goal final
        angle_to_target = math.atan2(dy, dx)
        alpha = (angle_to_target - theta + math.pi) % (2 * math.pi) - math.pi
        
        # 3. État ALIGNEMENT (Hystérésis sur cible locale)
        if not self._in_alignment_mode and abs(alpha) > math.radians(45):
            self._in_alignment_mode = True
        elif self._in_alignment_mode and abs(alpha) < math.radians(10):
            self._in_alignment_mode = False
        
        if self._in_alignment_mode:
            v = 0.0
            omega = -(1.2 * alpha)  # Signe inversé pour hardware
            omega_sat = np.clip(omega, -self.max_angular_vel, self.max_angular_vel)
            print(f"[CTRL] ALIGNMENT | alpha={math.degrees(alpha):.1f}° → ω={omega_sat:.3f}")
            return (v, omega_sat)
        
        # 4. Approche finale (seulement s'il reste <= 2 points)
        if dist_to_target < self.lookahead_distance and len(waypoints) <= 2:
            v = 0.4 * dist_to_target
            omega = -(1.0 * alpha)
            v_sat = np.clip(v, 0, self.max_linear_vel)
            omega_sat = np.clip(omega, -self.max_angular_vel, self.max_angular_vel)
            print(f"[CTRL] FINAL_APPROACH | dist={dist_to_target:.3f}m alpha={math.degrees(alpha):.1f}° → v={v_sat:.3f} ω={omega_sat:.3f}")
            return (v_sat, omega_sat)
        
        # 5. Pure Pursuit classique
        v, omega = self._pure_pursuit(current_pose, target_wp)
        print(f"[CTRL] PURE_PURSUIT | dist={dist_to_target:.3f}m → v={v:.3f} ω={omega:.3f}")
        return (v, omega)
    
    def _get_lookahead_point(self, pose, waypoints):
        """
        Trouve le waypoint à la distance de visée.
        
        Cherche le point le plus loin qui est à la distance lookahead
        et devant le robot (x_r > 0).
        
        Args:
            pose: Pose actuelle du robot
            waypoints: Liste de waypoints
            
        Returns:
            (x, y) waypoint cible
        """
        x, y, theta = pose
        
        # On cherche le point le plus loin qui est à la distance lookahead
        for wp in reversed(waypoints):
            dist = math.sqrt((wp[0] - x)**2 + (wp[1] - y)**2)
            if dist >= self.lookahead_distance:
                # Vérifier s'il est devant (x_r > 0)
                dx, dy = wp[0] - x, wp[1] - y
                x_r = math.cos(theta) * dx + math.sin(theta) * dy
                if x_r > 0:
                    return wp
        
        # Sinon retourner le dernier waypoint
        return waypoints[-1]
    
    def _pure_pursuit(self, pose, target):
        """
        Contrôleur Pure Pursuit géométrique.
        
        Args:
            pose: (x, y, theta) - robot pose in world frame
            target: (x_t, y_t) - target waypoint in world frame
            
        Returns:
            (v, omega) - linear and angular velocities
        """
        x, y, theta = pose
        dx, dy = target[0] - x, target[1] - y
        
        # Transformation repère robot
        x_r = math.cos(theta) * dx + math.sin(theta) * dy
        y_r = -math.sin(theta) * dx + math.cos(theta) * dy
        
        L_d2 = x_r**2 + y_r**2
        if L_d2 < 0.001:
            return (0.0, 0.0)
        
        # Courbure
        kappa = (2.0 * y_r) / L_d2
        
        # Vitesse avec modulation en courbe
        v = self.k_v * math.sqrt(L_d2)
        v = v / (1.0 + 0.5 * abs(kappa))  # Ralentir en courbe
        
        # Commande angulaire (signe inversé)
        omega = -(v * kappa)
        
        # Saturation
        v = np.clip(v, 0, self.max_linear_vel)
        omega = np.clip(omega, -self.max_angular_vel, self.max_angular_vel)
        
        return (v, omega)
    
    def is_waypoint_reached(self, pose, waypoint):
        """
        Vérifie si le waypoint actuel est atteint.
        
        Args:
            pose: (x, y, theta)
            waypoint: (x, y)
            
        Returns:
            True si dans le seuil
        """
        dist = math.sqrt((waypoint[0] - pose[0])**2 + (waypoint[1] - pose[1])**2)
        return dist < self.waypoint_reached_threshold
