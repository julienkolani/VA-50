"""
Cinématique - Modèle de mouvement robot

Cinématique de robot à entraînement différentiel pour Turtlebot Burger :
- Cinématique directe : (v, ω) -> (dx, dy, dθ)
- Cinématique inverse : contraintes de vitesse
- Modèle dynamique (simplifié)

Utilisé pour la simulation et la validation du contrôle.
"""

import numpy as np
from typing import Tuple


class DifferentialDriveKinematics:
    """
    Cinématique pour robot à entraînement différentiel (Turtlebot Burger).
    
    Paramètres du robot :
    - Empattement : distance entre les roues
    - Rayon des roues
    """
    
    def __init__(self, wheel_base: float = 0.16, wheel_radius: float = 0.033):
        """
        Initialise le modèle cinématique.
        
        Args:
            wheel_base: Distance entre les roues en mètres (Burger : 0.16m)
            wheel_radius: Rayon des roues en mètres (Burger : 0.033m)
        """
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        
    def forward_kinematics(self, 
                          v: float, 
                          omega: float, 
                          current_pose: Tuple[float, float, float],
                          dt: float) -> Tuple[float, float, float]:
        """
        Calcule la nouvelle pose à partir des commandes de vitesse.
        
        Args:
            v: Vitesse linéaire en m/s
            omega: Vitesse angulaire en rad/s
            current_pose: (x, y, theta) pose actuelle
            dt: Pas de temps en secondes
            
        Returns:
            (x_new, y_new, theta_new) pose mise à jour
            
        Équations :
            dx = v * cos(θ) * dt
            dy = v * sin(θ) * dt
            dθ = ω * dt
        """
        x, y, theta = current_pose
        
        # Mise à jour de l'orientation en premier
        theta_new = theta + omega * dt
        
        # Theta moyen pour une intégration plus précise
        theta_avg = theta + 0.5 * omega * dt
        
        # Mise à jour de la position
        x_new = x + v * np.cos(theta_avg) * dt
        y_new = y + v * np.sin(theta_avg) * dt
        
        # Normalisation de theta entre [-pi, pi]
        theta_new = np.arctan2(np.sin(theta_new), np.cos(theta_new))
        
        return (x_new, y_new, theta_new)
    
    def wheel_velocities_to_body(self, 
                                 v_left: float, 
                                 v_right: float) -> Tuple[float, float]:
        """
        Convertit les vitesses des roues en vitesses du corps.
        
        Args:
            v_left: Vitesse roue gauche en m/s
            v_right: Vitesse roue droite en m/s
            
        Returns:
            (v, omega): vitesses linéaire et angulaire du corps
        """
        v = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / self.wheel_base
        
        return (v, omega)
    
    def body_to_wheel_velocities(self, 
                                 v: float, 
                                 omega: float) -> Tuple[float, float]:
        """
        Convertit les vitesses du corps en vitesses des roues.
        
        Args:
            v: Vitesse linéaire en m/s
            omega: Vitesse angulaire en rad/s
            
        Returns:
            (v_left, v_right): vitesses des roues en m/s
        """
        v_left = v - (omega * self.wheel_base) / 2.0
        v_right = v + (omega * self.wheel_base) / 2.0
        
        return (v_left, v_right)
    
    def validate_velocities(self, 
                           v: float, 
                           omega: float,
                           max_v: float = 0.22,
                           max_omega: float = 2.84) -> Tuple[float, float]:
        """
        Valide et limite les vitesses aux contraintes du robot.
        
        Args:
            v, omega: Vitesses désirées
            max_v: Vitesse linéaire maximum (Burger : 0.22 m/s)
            max_omega: Vitesse angulaire maximum (Burger : 2.84 rad/s)
            
        Returns:
            (v_clamped, omega_clamped)
        """
        v_clamped = np.clip(v, -max_v, max_v)
        omega_clamped = np.clip(omega, -max_omega, max_omega)
        
        return (v_clamped, omega_clamped)
