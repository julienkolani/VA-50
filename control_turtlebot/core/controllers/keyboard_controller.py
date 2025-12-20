"""Contrôleur clavier pour TurtleBot."""

import pygame
import math
from typing import Tuple
import logging
from .base_controller import BaseController

logger = logging.getLogger(__name__)


class KeyboardController(BaseController):
    """
    Contrôleur clavier avec mouvement basé sur la physique.
    
    Contrôles :
        Flèches / WASD - Mouvement
        Espace - Arrêt d'urgence
        + / - - Ajuster le facteur de vitesse
    """
    
    def __init__(self, config: dict):
        """
        Initialise le contrôleur clavier.
        
        Args:
            config: Configuration clavier depuis controls.yaml
        """
        super().__init__(config)
        
        # Extract config
        kbd_cfg = config.get('keyboard', {})
        speed_cfg = config.get('speed_control', {})
        
        # Speed factor management
        self.speed_factor = speed_cfg.get('initial_factor', 0.07)
        self.min_factor = speed_cfg.get('min_factor', 0.2)
        self.max_factor = speed_cfg.get('max_factor', 2.0)
        self.factor_increment = speed_cfg.get('increment', 0.1)
        
        # Physics parameters (scaled by speed factor)
        self.base_accel_linear = kbd_cfg.get('acceleration_linear', 0.2)
        self.base_accel_angular = kbd_cfg.get('acceleration_angular', 2.5)
        self.base_max_linear = kbd_cfg.get('max_linear_mps', 3.5)
        self.base_max_angular = kbd_cfg.get('max_angular_dps', 120)
        
        # Apply scaling
        self._update_scaled_params()
        
        # Friction
        self.friction_linear = kbd_cfg.get('friction_linear', 0.92)
        self.friction_angular = kbd_cfg.get('friction_angular', 0.82)
        
        # Current velocities (in display units)
        self.velocity_linear = 0.0
        self.velocity_angular = 0.0
    
    def _update_scaled_params(self):
        """Update parameters based on current speed factor."""
        self.accel_linear = self.base_accel_linear * self.speed_factor
        self.accel_angular = self.base_accel_angular * self.speed_factor
        self.max_linear = self.base_max_linear * self.speed_factor
        self.max_angular = self.base_max_angular * self.speed_factor
    
    def increase_speed(self):
        """Increase speed factor."""
        old_factor = self.speed_factor
        self.speed_factor = min(self.speed_factor + self.factor_increment, self.max_factor)
        
        if self.speed_factor != old_factor:
            ratio = self.speed_factor / old_factor
            self.velocity_linear *= ratio
            self.velocity_angular *= ratio
            self._update_scaled_params()
            logger.info(f"Facteur de vitesse augmenté à {self.speed_factor:.2f}")
    
    def decrease_speed(self):
        """Decrease speed factor."""
        old_factor = self.speed_factor
        self.speed_factor = max(self.speed_factor - self.factor_increment, self.min_factor)
        
        if self.speed_factor != old_factor:
            ratio = self.speed_factor / old_factor
            self.velocity_linear *= ratio
            self.velocity_angular *= ratio
            self._update_scaled_params()
            logger.info(f"Facteur de vitesse diminué à {self.speed_factor:.2f}")
    
    def update(self, events: list) -> None:
        """
        Met à jour le contrôleur basé sur l'entrée clavier.
        
        Args:
            events: Liste des événements pygame
        """
        if not self._enabled:
            return
        
        # Get current key states
        keys = pygame.key.get_pressed()
        
        # Linear velocity control
        if keys[pygame.K_UP] or keys[pygame.K_w]:
            self.velocity_linear = min(
                self.velocity_linear + self.accel_linear,
                self.max_linear
            )
        elif keys[pygame.K_DOWN] or keys[pygame.K_s]:
            self.velocity_linear = max(
                self.velocity_linear - self.accel_linear,
                -self.max_linear
            )
        else:
            # Apply friction
            self.velocity_linear *= self.friction_linear
            if abs(self.velocity_linear) < 0.05:
                self.velocity_linear = 0.0
        
        # Angular velocity control with direction correction for reverse
        left = keys[pygame.K_LEFT] or keys[pygame.K_a]
        right = keys[pygame.K_RIGHT] or keys[pygame.K_d]
        forward = keys[pygame.K_UP] or keys[pygame.K_w]
        backward = keys[pygame.K_DOWN] or keys[pygame.K_s]
        
        # Direction factor inverts turning when moving backward
        direction_factor = -1 if self.velocity_linear < 0 else 1
        
        if left and right:
            # Both directions pressed - special case for sharp turning
            if forward:
                self.velocity_angular = self.max_angular
            elif backward:
                self.velocity_angular = -self.max_angular
            else:
                self.velocity_angular *= self.friction_angular
        elif left:
            # Turn left (corrected for reverse)
            intensity = 1.0 + abs(self.velocity_linear) * 0.3
            self.velocity_angular = max(
                self.velocity_angular - self.accel_angular * intensity * direction_factor,
                -self.max_angular * 0.6
            )
        elif right:
            # Turn right (corrected for reverse)
            intensity = 1.0 + abs(self.velocity_linear) * 0.3
            self.velocity_angular = min(
                self.velocity_angular + self.accel_angular * intensity * direction_factor,
                self.max_angular * 0.6
            )
        else:
            # Apply friction
            self.velocity_angular *= self.friction_angular
            if abs(self.velocity_angular) < 0.2:
                self.velocity_angular = 0.0
        
        # Convert to ROS command (m/s and rad/s)
        self.linear_cmd = self.velocity_linear * 0.1
        self.angular_cmd = math.radians(self.velocity_angular)
        
        # Handle discrete events (speed adjustment, emergency stop)
        for event in events:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                    self.increase_speed()
                elif event.key == pygame.K_MINUS:
                    self.decrease_speed()
                elif event.key == pygame.K_SPACE:
                    self.emergency_stop()
    
    def get_command(self) -> Tuple[float, float]:
        """
        Obtient la commande de vitesse actuelle.
        
        Returns:
            Tuple de (vitesse_lineaire_m/s, vitesse_angulaire_rad/s)
        """
        if not self._enabled:
            return (0.0, 0.0)
        return (self.linear_cmd, self.angular_cmd)
    
    def emergency_stop(self) -> None:
        """Exécute l'arrêt d'urgence."""
        self.velocity_linear = 0.0
        self.velocity_angular = 0.0
        self.linear_cmd = 0.0
        self.angular_cmd = 0.0
        logger.warning("Arrêt d'urgence déclenché")
    
    def get_speed_factor(self) -> float:
        """Obtient le facteur de vitesse actuel."""
        return self.speed_factor
