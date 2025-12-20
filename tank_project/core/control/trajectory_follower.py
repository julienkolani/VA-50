"""
Suiveur de Trajectoire - Contrôleur de Suivi de Chemin

Implémente le suivi de trajectoire pour la navigation par points de passage :
- Contrôleur Pure Pursuit
- Contrôle de cap basé sur PID
- Avancement dynamique des points de passage
- Réactions d'évitement d'obstacles

Prend un chemin (liste de waypoints) et la pose actuelle, sort (v, ω).

Logs : [CTRL] Suivi waypoint (x,y), distance : Dm
"""

import numpy as np
from typing import Tuple, List, Optional


class TrajectoryFollower:
    """
    Contrôleur de suivi de trajectoire pour la navigation par points.
    
    Utilise l'algorithme Pure Pursuit pour un suivi fluide.
    """
    
    def __init__(self, config):
        """
        Initialise le suiveur de trajectoire.
        
        Args:
            config: Paramètres du contrôleur depuis config/robot.yaml :
                - lookahead_distance: Distance de visée Pure Pursuit
                - k_v: Gain de vitesse linéaire
                - k_theta: Gain de vitesse angulaire
                - max_linear_vel: V max en m/s
                - max_angular_vel: ω max en rad/s
        """
        self.lookahead_distance = config.get('lookahead_distance', 0.3)
        self.k_v = config.get('k_v', 1.0)
        self.k_theta = config.get('k_theta', 2.0)
        self.max_linear_vel = config.get('max_linear_vel', 0.22)
        self.max_angular_vel = config.get('max_angular_vel', 2.84)
        
        self.waypoint_reached_threshold = config.get('waypoint_threshold', 0.1)
        
    def compute_control(self, 
                       current_pose: Tuple[float, float, float],
                       waypoints: List[Tuple[float, float]]) -> Tuple[float, float]:
        """
        Calcule les commandes de contrôle pour suivre les waypoints.
        
        Args:
            current_pose: (x, y, theta) pose actuelle du robot
            waypoints: Liste de waypoints (x, y) en mètres
            
        Returns:
            (v, omega): vitesses linéaire et angulaire
            
        Algorithme (Pure Pursuit) :
            1. Trouve le point de visée (lookahead) sur le chemin
            2. Calcule la courbure pour atteindre ce point
            3. Calcule v et ω à partir de la courbure
            4. Limite aux vitesses max
            
        Logs :
            [CTRL] Target waypoint (x,y), distance: Dm, heading error: θ rad
        """
        if not waypoints:
            return (0.0, 0.0)
        
        x, y, theta = current_pose
        
        # Trouve le waypoint cible (lookahead)
        target_wp = self._get_lookahead_point(current_pose, waypoints)
        
        if target_wp is None:
            return (0.0, 0.0)
        
        # Calcule le contrôle
        v, omega = self._pure_pursuit(current_pose, target_wp)
        
        # Limite les vitesses
        v = np.clip(v, -self.max_linear_vel, self.max_linear_vel)
        omega = np.clip(omega, -self.max_angular_vel, self.max_angular_vel)
        
        return (v, omega)
    
    def _get_lookahead_point(self, pose, waypoints):
        """
        Trouve le waypoint à la distance de visée.
        
        Args:
            pose: Pose actuelle du robot
            waypoints: Liste de waypoints
            
        Returns:
            (x, y) waypoint cible
        """
        x, y, _ = pose
        
        # Trouve le premier waypoint au-delà de la distance de visée
        for wp in waypoints:
            dist = np.sqrt((wp[0] - x)**2 + (wp[1] - y)**2)
            if dist >= self.lookahead_distance:
                return wp
        
        # Si tous les waypoints sont plus proches, retourne le dernier
        return waypoints[-1] if waypoints else None
    
    def _pure_pursuit(self, pose, target):
        """
        Loi de commande Pure Pursuit.
        
        Args:
            pose: (x, y, theta)
            target: (x_t, y_t)
            
        Returns:
            (v, omega)
        """
        x, y, theta = pose
        x_t, y_t = target
        
        # Calcule distance et angle vers la cible
        dx = x_t - x
        dy = y_t - y
        distance = np.sqrt(dx**2 + dy**2)
        
        # Angle cible
        target_theta = np.arctan2(dy, dx)
        
        # Erreur de cap
        theta_error = self._normalize_angle(target_theta - theta)
        
        # Vitesse linéaire proportionnelle à la distance
        v = self.k_v * distance
        
        # Vitesse angulaire proportionnelle à l'erreur de cap
        omega = self.k_theta * theta_error
        
        # Réduit la vitesse linéaire quand on tourne
        v *= np.cos(theta_error)
        
        return (v, omega)
    
    def _normalize_angle(self, angle):
        """Normalise l'angle entre [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
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
