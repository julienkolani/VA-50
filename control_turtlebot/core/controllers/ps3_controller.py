"""Contrôleur PS3 pour TurtleBot."""

import pygame
import math
from typing import Tuple
import logging
from .base_controller import BaseController

logger = logging.getLogger(__name__)


class PS3Controller(BaseController):
    """
    Contrôleur manette PS3 avec deadzone et mapping de boutons.
    
    Fonctionnalités :
    - Contrôle stick analogique
    - Surcharge D-pad
    - Compensation deadzone
    - Support Hot-plug
    """
    
    def __init__(self, config: dict):
        """
        Initialise le contrôleur PS3.
        
        Args:
            config: Configuration PS3 depuis controls.yaml
        """
        super().__init__(config)
        
        # Extract config
        ps3_cfg = config.get('ps3', {})
        speed_cfg = config.get('speed_control', {})
        
        # Speed factor management
        self.speed_factor = speed_cfg.get('initial_factor', 0.07)
        
        # Max velocities (scaled by speed factor)
        self.base_max_linear = ps3_cfg.get('max_linear_mps', 3.5)
        self.base_max_angular = ps3_cfg.get('max_angular_dps', 120)
        self._update_scaled_params()
        
        # Deadzone
        self.deadzone = ps3_cfg.get('deadzone', 0.15)
        
        # Button and axis mapping
        button_map = ps3_cfg.get('button_mapping', {})
        axis_map = ps3_cfg.get('axis_mapping', {})
        
        self.btn_emergency_stop = button_map.get('emergency_stop', 0)
        self.axis_left_x = axis_map.get('left_x', 0)
        self.axis_left_y = axis_map.get('left_y', 1)
        self.axis_right_x = axis_map.get('right_x', 2)
        
        # Joystick state
        self.joystick = None
        self.connected = False
        
        # Initialize pygame joystick system
        pygame.joystick.init()
        self._detect_controller()
    
    def _update_scaled_params(self):
        """Update parameters based on current speed factor."""
        self.max_linear = self.base_max_linear * self.speed_factor
        self.max_angular = self.base_max_angular * self.speed_factor
    
    def _detect_controller(self):
        """Detect and initialize first available joystick."""
        joystick_count = pygame.joystick.get_count()
        
        if joystick_count == 0:
            logger.warning("Aucun joystick détecté")
            self.connected = False
            return False
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.connected = True
        
        logger.info(f"Joystick connecté : {self.joystick.get_name()}")
        logger.info(f"  Axes : {self.joystick.get_numaxes()}, Boutons : {self.joystick.get_numbuttons()}")
        
        return True
    
    def _apply_deadzone(self, value: float) -> float:
        """
        Applique la deadzone à la valeur de l'axe pour éliminer le drift.
        
        Args:
            value: Valeur brute de l'axe [-1, 1]
            
        Returns:
            Valeur corrigée avec deadzone appliquée
        """
        if abs(value) < self.deadzone:
            return 0.0
        
        # Rescale to maintain full range after deadzone
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def set_speed_factor(self, factor: float):
        """
        Définit le facteur de vitesse.
        
        Args:
            factor: Nouveau facteur de vitesse
        """
        self.speed_factor = factor
        self._update_scaled_params()
        logger.info(f"Facteur de vitesse défini à {factor:.2f}")
    
    def update(self, events: list) -> None:
        """
        Met à jour le contrôleur basé sur l'entrée joystick.
        
        Args:
            events: Liste des événements pygame
        """
        if not self._enabled or not self.connected or self.joystick is None:
            self.linear_cmd = 0.0
            self.angular_cmd = 0.0
            return
        
        # Read analog stick axes
        axis_x = self.joystick.get_axis(self.axis_left_x)  # left/right
        axis_y = self.joystick.get_axis(self.axis_left_y)  # forward/backward
        
        # Apply deadzone
        axis_x = self._apply_deadzone(axis_x)
        axis_y = self._apply_deadzone(axis_y)
        
        # PS3 stick: up = negative value
        velocity_linear = -axis_y * self.max_linear
        
        # Direction correction for reverse (same as keyboard)
        direction_factor = -1 if velocity_linear < 0 else 1
        velocity_angular = -axis_x * self.max_angular * direction_factor
        
        # D-pad override (higher priority)
        try:
            if self.joystick.get_numhats() > 0:
                hat_x, hat_y = self.joystick.get_hat(0)
                
                # Forward/backward override
                if hat_y != 0:
                    velocity_linear = hat_y * self.max_linear
                    direction_factor = -1 if velocity_linear < 0 else 1
                
                # Left/right override
                if hat_x != 0:
                    velocity_angular = -hat_x * self.max_angular * direction_factor
        except Exception as e:
            logger.debug(f"D-pad read error: {e}")
        
        # Convert to ROS commands (m/s and rad/s)
        self.linear_cmd = velocity_linear * 0.1
        self.angular_cmd = math.radians(velocity_angular)
        
        # Handle events (emergency stop, hotplug)
        for event in events:
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == self.btn_emergency_stop:
                    self.emergency_stop()
            elif event.type == pygame.JOYDEVICEREMOVED:
                logger.warning("Joystick déconnecté")
                self.connected = False
                self.joystick = None
            elif event.type == pygame.JOYDEVICEADDED:
                logger.info("Nouveau joystick détecté")
                self._detect_controller()
    
    def get_command(self) -> Tuple[float, float]:
        """
        Obtient la commande de vitesse actuelle.
        
        Returns:
            Tuple de (vitesse_lineaire_m/s, vitesse_angulaire_rad/s)
        """
        if not self._enabled or not self.connected:
            return (0.0, 0.0)
        return (self.linear_cmd, self.angular_cmd)
    
    def emergency_stop(self) -> None:
        """Exécute l'arrêt d'urgence."""
        self.linear_cmd = 0.0
        self.angular_cmd = 0.0
        logger.warning("Arrêt d'urgence déclenché (PS3)")
    
    def is_connected(self) -> bool:
        """Vérifie si le joystick est connecté."""
        return self.connected
