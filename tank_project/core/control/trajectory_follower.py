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
        
        # Support pour ancienne (flat) et nouvelle (nested) structure
        # Nouvelle structure : config est le dict 'control' complet
        
        # 1. PARAMÈTRES DE BASE (Pure Pursuit)
        self.lookahead_distance = config.get('lookahead_distance', 0.15)
        self.k_v = config.get('k_v', 0.6)
        
        # 2. LIMITES
        limits = config.get('limits', {})
        # Retour aux clés plates si le dictionnaire 'limits' est manquant
        max_lin = limits.get('max_linear_vel', config.get('max_linear_vel', 0.22))
        max_ang = limits.get('max_angular_vel', config.get('max_angular_vel', 1.2))
        
        self.max_linear_vel = float(max_lin)
        self.max_angular_vel = float(max_ang)
        
        # 3. ALIGNEMENT (Rotation sur place)
        align = config.get('alignment', {})
        self.align_threshold = math.radians(align.get('threshold_angle', 45.0))
        self.align_hysteresis = math.radians(align.get('hysteresis_angle', 10.0))
        self.gain_omega_align = align.get('gain_omega', 1.2)
        
        # 4. APPROCHE FINALE (Douceur)
        final = config.get('final_approach', {})
        self.final_gain_v = final.get('gain_v', 0.4)
        self.final_gain_omega = final.get('gain_omega', 1.0)
        
        # 5. PRÉCISION
        eps = config.get('epsilon', {})
        self.waypoint_reached_threshold = eps.get('waypoint_threshold', config.get('waypoint_threshold', 0.07))
    
    def compute_control(self, current_pose: Tuple[float, float, float], waypoints: List[Tuple[float, float]]) -> Tuple[float, float]:
        """
        Calcule les commandes de contrôle (v, omega) basées sur l'état actuel.
        
        Implémente la Machine à États Hybride :
        1. Trouver le Point Lookahead.
        2. Calculer l'Erreur de Cap Locale (Alpha).
        3. Vérifier le Mode Alignement (Le Fix pour le bug 180°).
        4. Vérifier l'Approche Finale (Précision).
        5. Exécuter Pure Pursuit.
        """
        if not waypoints:
            return (0.0, 0.0)
        
        x, y, theta = current_pose
        
        # 1. Trouve le point cible (Lookahead)
        target_wp = self._get_lookahead_point(current_pose, waypoints)
        dx = target_wp[0] - x
        dy = target_wp[1] - y
        dist_to_target = math.sqrt(dx**2 + dy**2)
        
        # 2. Alpha calculé sur la CIBLE LOCALE (lookahead), pas le but final
        angle_to_target = math.atan2(dy, dx)
        alpha = (angle_to_target - theta + math.pi) % (2 * math.pi) - math.pi
        
        # 3. ÉTAT ALIGNEMENT (Hystérésis - Fix Historique pour le spin 180)
        # Si on tourne le dos à la cible : STOP et Rotation.
        if not self._in_alignment_mode and abs(alpha) > self.align_threshold:
            self._in_alignment_mode = True
        elif self._in_alignment_mode and abs(alpha) < self.align_hysteresis:
            self._in_alignment_mode = False
        
        if self._in_alignment_mode:
            v = 0.0 # Stop mouvement linéaire
            omega = -(self.gain_omega_align * alpha)  # Pivoter pour faire face
            omega_sat = np.clip(omega, -self.max_angular_vel, self.max_angular_vel)
            # Log pour débuguer l'hystérésis
            print(f"[CTRL] ALIGNEMENT | alpha={math.degrees(alpha):.1f}° → ω={omega_sat:.3f}")
            return (v, omega_sat)
        
        # 4. Approche finale (seulement s'il reste <= 2 points)
        if dist_to_target < self.lookahead_distance and len(waypoints) <= 2:
            v = self.final_gain_v * dist_to_target
            omega = -(self.final_gain_omega * alpha)
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
