"""
Suiveur de Trajectoire - Contrôleur Pure Pursuit Géométrique

Implémente le suivi de trajectoire par courbure géométrique :
- Sélection du point lookahead sur le chemin
- Transformation monde → repère robot
- Calcul de courbure : κ = 2*y_r / L_d²
- Commande angulaire : ω = v * κ

Prend un chemin (liste de waypoints) et la pose actuelle, sort (v, ω).
"""

import numpy as np
from typing import Tuple, List, Optional


class TrajectoryFollower:
    """
    Contrôleur de suivi de trajectoire pour la navigation par points.
    
    Utilise l'algorithme Pure Pursuit pour un suivi fluide.
    """
    
    def __init__(self, config):
        # État interne pour hystérésis
        self._in_alignment_mode = False
        """
        Initialise le suiveur de trajectoire.
        
        Args:
            config: Paramètres du contrôleur depuis config/robot.yaml :
                - lookahead_distance_m: Distance de visée Pure Pursuit
                - k_velocity: Gain de vitesse linéaire
                - max_linear_mps: Vitesse linéaire maximale
                - max_angular_radps: Vitesse angulaire maximale
                - waypoint_threshold_m: Seuil waypoint atteint
        """
        # Mapping flexible pour supporter les deux conventions de nommage
        self.lookahead_distance = config.get('lookahead_distance_m', 
                                              config.get('lookahead_distance', 0.3))
        self.k_v = config.get('k_velocity', config.get('k_v', 1.0))

        # Support new convention (mps) and legacy (vel)
        max_lin = config.get('max_linear_mps', config.get('max_linear_vel', 0.22))
        max_ang = config.get('max_angular_radps', config.get('max_angular_vel', 2.84))
        
        self.max_linear_vel = float(max_lin)
        self.max_angular_vel = float(max_ang)
        
        # Pure Pursuit: forward-only motion (no reverse)
        self.min_linear_vel = 0.0
        
        self.waypoint_reached_threshold = config.get('waypoint_threshold_m',
                                                      config.get('waypoint_threshold', 0.1))
        
    def compute_control(self, 
                       current_pose: Tuple[float, float, float],
                       waypoints: List[Tuple[float, float]]) -> Tuple[float, float]:
        """
        Calcule les commandes de contrôle pour suivre les waypoints.
        
        Machine d'état :
        - STOP: Distance < seuil → arrêt complet
        - GOAL_APPROACH: Distance < lookahead → contrôleur terminal (rho/alpha)
        - INITIAL_ALIGN: Cible >90° derrière → rotation sur place
        - PATH_FOLLOW: Sinon → Pure Pursuit géométrique
        
        Args:
            current_pose: (x, y, theta) pose actuelle du robot
            waypoints: Liste de waypoints (x, y) en mètres
            
        Returns:
            (v, omega): vitesses linéaire et angulaire
        """
        if not waypoints:
            return (0.0, 0.0)
        
        import math
        x, y, theta = current_pose
        
        # Dernier waypoint = goal
        goal = waypoints[-1]
        dx_goal = goal[0] - x
        dy_goal = goal[1] - y
        rho = math.sqrt(dx_goal**2 + dy_goal**2)  # Distance to goal
        angle_goal = math.atan2(dy_goal, dx_goal)
        alpha = (angle_goal - theta + math.pi) % (2 * math.pi) - math.pi
        
        # ========== STATE 1: STOP ==========
        if rho < self.waypoint_reached_threshold:
            self._in_alignment_mode = False  # Reset alignment flag
            print(f"[CTRL] STATE: STOP | rho={rho:.3f}m < threshold")
            return (0.0, 0.0)
        
        # ========== STATE 2: ALIGNMENT (Hystérésis) ==========
        # CRITIQUE: Vérifier l'alignement AVANT d'approcher la cible
        # Empêche spiraling en forçant rotation sur place si mal aligné
        align_enter_threshold = math.radians(45)
        align_exit_threshold = math.radians(10)
        
        # Mise à jour de l'état avec hystérésis
        if not self._in_alignment_mode and abs(alpha) > align_enter_threshold:
            self._in_alignment_mode = True
        elif self._in_alignment_mode and abs(alpha) < align_exit_threshold:
            self._in_alignment_mode = False
        
        if self._in_alignment_mode:
            v = 0.0
            k_align = 1.5  # Gain de rotation sur place
            omega = np.clip(k_align * alpha, -self.max_angular_vel, self.max_angular_vel)
            print(f"[CTRL] STATE: ALIGNMENT | alpha={math.degrees(alpha):.1f}° → ω={omega:.3f}")
            return (v, omega)
        
        # ========== STATE 3: GOAL_APPROACH ==========
        # Seuil réduit (0.15m) pour laisser Pure Pursuit travailler plus longtemps
        if rho < 0.15:
            # Contrôleur unicycle terminal avec gains doux
            k_rho = 0.3   # Approche très douce
            k_alpha = 1.0  # Évite les coups de volant
            
            v = k_rho * rho
            omega = k_alpha * alpha
            
            # Saturation
            v = max(min(v, self.max_linear_vel), self.min_linear_vel)
            omega = max(min(omega, self.max_angular_vel), -self.max_angular_vel)
            
            print(f"[CTRL] STATE: GOAL_APPROACH | rho={rho:.3f}m alpha={math.degrees(alpha):.1f}° → v={v:.3f} ω={omega:.3f}")
            return (v, omega)
        
        # ========== STATE 4: PATH_FOLLOW (Pure Pursuit) ==========
        target_wp = self._get_lookahead_point(current_pose, waypoints)
        
        if target_wp is None:
            return (0.0, 0.0)
        
        v, omega = self._pure_pursuit(current_pose, target_wp)
        
        print(f"[CTRL] STATE: PATH_FOLLOW | rho={rho:.3f}m → v={v:.3f} ω={omega:.3f}")
        return (v, omega)
    
    def _get_lookahead_point(self, pose, waypoints):
        """
        Trouve le waypoint à la distance de visée.
        
        CRITIQUE: Le waypoint doit être DEVANT le robot (x_r > 0)
        pour éviter l'oscillation autour de x_r ≈ 0.
        
        Args:
            pose: Pose actuelle du robot
            waypoints: Liste de waypoints
            
        Returns:
            (x, y) waypoint cible
        """
        import math
        
        x, y, theta = pose
        
        # Trouve le premier waypoint devant ET au-delà de la distance de visée
        for wp in waypoints:
            dx = wp[0] - x
            dy = wp[1] - y
            
            # Transform to robot frame to check if in front
            x_r = math.cos(theta) * dx + math.sin(theta) * dy
            
            # Skip waypoints behind robot
            if x_r <= 0:
                continue
            
            dist = math.sqrt(dx**2 + dy**2)
            if dist >= self.lookahead_distance:
                return wp
        
        # Si aucun waypoint valide, chercher le dernier waypoint devant
        for wp in reversed(waypoints):
            dx = wp[0] - x
            dy = wp[1] - y
            x_r = math.cos(theta) * dx + math.sin(theta) * dy
            if x_r > 0:
                return wp
        
        # Dernier recours: retourner le dernier waypoint (même s'il est derrière)
        return waypoints[-1] if waypoints else None
    
    def _pure_pursuit(self, pose, target):
        """
        Canonical Geometric Pure Pursuit Controller.
        
        Implements pure pursuit using curvature-based control law:
        - Transform target to robot frame
        - Compute curvature: κ = 2*y_r / L_d²
        - Compute angular velocity: ω = v * κ
        
        This produces continuous, stable commands without rotation-on-the-spot.
        
        Args:
            pose: (x, y, theta) - robot pose in world frame
            target: (x_t, y_t) - target waypoint in world frame
            
        Returns:
            (v, omega) - linear and angular velocities
        """
        import math
        
        x, y, theta = pose
        x_t, y_t = target
        
        # Step 1: Transform target from world frame to robot frame
        dx = x_t - x
        dy = y_t - y
        
        # Robot frame transformation
        x_r = math.cos(theta) * dx + math.sin(theta) * dy
        y_r = -math.sin(theta) * dx + math.cos(theta) * dy
        
        # Debug: log frame transformation
        print(f"[CTRL] Pure Pursuit: x_r={x_r:.3f} y_r={y_r:.3f} L_d={math.sqrt(x_r**2 + y_r**2):.3f}")
        
        # Safety: if target somehow behind (shouldn't happen with alignment mode)
        if x_r <= 0:
            print(f"[CTRL] WARNING: x_r <= 0 despite alignment mode, stopping")
            return (0.0, 0.0)
        
        # Step 2: Compute lookahead distance (actual distance to target)
        L_d = math.sqrt(x_r**2 + y_r**2)
        
        # Step 3: Compute curvature using geometric Pure Pursuit formula
        # κ = 2 * y_r / L_d²
        if L_d < 0.01:  # Too close, stop
            return (0.0, 0.0)
        
        kappa = (2.0 * y_r) / (L_d * L_d)
        
        # Step 4: Compute linear velocity with curvature modulation
        # Reduce speed in sharp turns for stability
        v = self.k_v * L_d
        
        # Modulation: v_final = v / (1 + factor * |kappa|)
        curvature_factor = 0.5  # Ajustable selon besoin
        v = v / (1.0 + curvature_factor * abs(kappa))
        
        # Step 5: Compute angular velocity from curvature
        # ω = v * κ
        omega = v * kappa
        
        # Step 6: Apply velocity limits (single saturation point)
        v = max(min(v, self.max_linear_vel), self.min_linear_vel)
        omega = max(min(omega, self.max_angular_vel), -self.max_angular_vel)
        
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
        x, y, _ = pose
        dist = np.sqrt((waypoint[0] - x)**2 + (waypoint[1] - y)**2)
        return dist < self.waypoint_reached_threshold
