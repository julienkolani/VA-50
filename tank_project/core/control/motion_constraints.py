"""
Contraintes de Mouvement - Limites de Vitesse & Accélération

Applique les contraintes physiques du robot :
- Vitesses maximales (linéaire, angulaire)
- Accélérations maximales
- Lissage de la vitesse
- Logique d'arrêt d'urgence

Empêche les commandes dangereuses et assure un mouvement fluide.
"""

import numpy as np
from typing import Tuple


class MotionConstraints:
    """
    Applique les contraintes physiques de mouvement pour un contrôle sûr du robot.
    
    Prévient :
    - Dépassement des limites de vitesse
    - Accélération excessive (changements brusques)
    - Commandes dangereuses
    """
    
    def __init__(self, config):
        """
        Initialise les contraintes de mouvement.
        
        Args:
            config: Configuration du robot :
                - max_linear_vel: m/s
                - max_angular_vel: rad/s
                - max_linear_accel: m/s²
                - max_angular_accel: rad/s²
        """
        self.max_v = config.get('max_linear_vel', 0.22)
        self.max_omega = config.get('max_angular_vel', 2.84)
        self.max_accel_v = config.get('max_linear_accel', 0.5)
        self.max_accel_omega = config.get('max_angular_accel', 5.0)
        
        # Commandes précédentes pour limitation d'accélération
        self.prev_v = 0.0
        self.prev_omega = 0.0
        
    def apply_constraints(self, 
                         v_desired: float, 
                         omega_desired: float,
                         dt: float) -> Tuple[float, float]:
        """
        Applique toutes les contraintes de mouvement.
        
        Args:
            v_desired: Vitesse linéaire désirée
            omega_desired: Vitesse angulaire désirée
            dt: Temps depuis la dernière commande (secondes)
            
        Returns:
            (v_safe, omega_safe): vitesses contraintes
            
        Étapes :
            1. Limite aux vitesses max
            2. Limite l'accélération
            3. Met à jour les commandes précédentes
        """
        # Limites de vitesse
        v = np.clip(v_desired, -self.max_v, self.max_v)
        omega = np.clip(omega_desired, -self.max_omega, self.max_omega)
        
        # Limites d'accélération
        if dt > 0:
            v = self._limit_acceleration(v, self.prev_v, self.max_accel_v, dt)
            omega = self._limit_acceleration(omega, self.prev_omega, 
                                            self.max_accel_omega, dt)
        
        # Stockage pour la prochaine itération
        self.prev_v = v
        self.prev_omega = omega
        
        return (v, omega)
    
    def _limit_acceleration(self, 
                           desired: float, 
                           previous: float,
                           max_accel: float, 
                           dt: float) -> float:
        """
        Limite l'accélération d'une composante de vitesse.
        
        Args:
            desired: Vitesse désirée
            previous: Vitesse précédente
            max_accel: Accélération maximum autorisée
            dt: Pas de temps
            
        Returns:
            Vitesse limitée en accélération
        """
        delta = desired - previous
        max_delta = max_accel * dt
        
        if abs(delta) > max_delta:
            delta = np.sign(delta) * max_delta
        
        return previous + delta
    
    def emergency_stop(self):
        """
        Réinitialise la vitesse à zéro immédiatement.
        
        Utilisé pour un arrêt de sécurité.
        """
        self.prev_v = 0.0
        self.prev_omega = 0.0
        
        return (0.0, 0.0)
    
    def soft_stop(self, dt: float) -> Tuple[float, float]:
        """
        Décélère graduellement jusqu'à zéro.
        
        Args:
            dt: Pas de temps
            
        Returns:
            Vitesses en décélération
        """
        return self.apply_constraints(0.0, 0.0, dt)
