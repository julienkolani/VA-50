#!/usr/bin/env python3
"""
Controleur manette PS3 pour TurtleBot.

Ce module gere les entrees d'une manette de jeu PS3 (ou compatible)
et les convertit en commandes de vitesse pour le robot. Il supporte
la detection automatique, la gestion des zones mortes (deadzone),
et les boutons speciaux (arret d'urgence).

Classes:
    PS3Controller: Controleur manette PS3 configurable.
"""

import pygame
import math



class PS3Controller:
    """Contrôleur manette PS3 configurable pour TurtleBot"""

    def __init__(self, config):
        """
        Initialise le contrôleur PS3.
        Args:
            config (dict): Configuration globale
        """
        pygame.joystick.init()
        self.joystick = None
        self.connected = False
        
        ctrl_cfg = config.get('controls', {})
        ps3_cfg = ctrl_cfg.get('ps3', {})
        speed_cfg = ctrl_cfg.get('speed_control', {})
        
        # Facteur de vitesse initial
        self.vitesse_factor = speed_cfg.get('initial_factor', 1.0)
        
        # Mappings
        self.btn_map = ps3_cfg.get('button_mapping', {})
        self.axis_map = ps3_cfg.get('axis_mapping', {})
        
        # Config des axes / boutons (fallback si mapping vide ou clé manquante)
        self.BTN_X = self.btn_map.get('cross', 0)
        self.BTN_CIRCLE = self.btn_map.get('circle', 1)
        self.BTN_TRIANGLE = self.btn_map.get('triangle', 2)
        self.BTN_SQUARE = self.btn_map.get('square', 3)

        self.AXIS_LEFT_X = self.axis_map.get('left_x', 0)
        self.AXIS_LEFT_Y = self.axis_map.get('left_y', 1)
        self.AXIS_RIGHT_X = self.axis_map.get('right_x', 2)
        
        self.deadzone = ps3_cfg.get('deadzone', 0.15)
        
        # Rampes de lissage
        self.ramp_linear = speed_cfg.get('acceleration_ramp_linear', 0.01)
        self.ramp_angular = speed_cfg.get('acceleration_ramp_angular', 0.05)

        # -- Définition des VITESSES MAX --
        # La config robot contient directement velocity_limits (pas imbriqué)
        robot_cfg = config.get('robot', {})
        robot_limits = robot_cfg.get('velocity_limits', {})
        
        # 1. Limite Absolue (Physique)
        self.limit_linear_mps = robot_limits.get('max_linear_mps', 0.22)
        self.limit_angular_radps = robot_limits.get('max_angular_radps', 4.2)
        
        # 2. Vitesse Max Contrôleur
        self.vitesse_max_lineaire = self.limit_linear_mps * self.vitesse_factor
        self.vitesse_max_angulaire = math.degrees(self.limit_angular_radps) * self.vitesse_factor
        
        # Limites strictes
        self.strict_max_linear = self.limit_linear_mps
        self.strict_max_angular = math.degrees(self.limit_angular_radps)
        
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.tir_demande = False
        
        self._detect_controller()


    # ================================================================
    #   Détection et gestion du joystick
    # ================================================================
    def _detect_controller(self):
        """Détecte et initialise la première manette disponible."""
        joystick_count = pygame.joystick.get_count()

        if joystick_count == 0:
            print("[PS3] Aucune manette detectee")
            self.connected = False
            return False

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        self.connected = True
        print(f"[PS3] Manette connectee : {self.joystick.get_name()}")
        print(f"   Axes: {self.joystick.get_numaxes()}")
        print(f"   Boutons: {self.joystick.get_numbuttons()}")

        return True


    # ================================================================
    #   Ajustement dynamique du facteur de vitesse (+ / -)
    # ================================================================
    # ================================================================
    #   Ajustement dynamique du facteur de vitesse (+ / -)
    # ================================================================
    def set_vitesse_factor(self, factor):

        ratio = factor / self.vitesse_factor
        self.vitesse_factor = factor

        # Recalcul des vitesses max basées sur les limites physiques
        self.vitesse_max_lineaire = self.limit_linear_mps * self.vitesse_factor
        self.vitesse_max_angulaire = math.degrees(self.limit_angular_radps) * self.vitesse_factor


    # ================================================================
    #   Deadzone
    # ================================================================
    def _apply_deadzone(self, value):
        """Supprime le drift du joystick."""
        if abs(value) < self.deadzone:
            return 0.0

        sign = 1 if value > 0 else -1
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)


    # ================================================================
    #   Mise à jour des commandes
    # ================================================================
    # ================================================================
    #   Mise à jour des commandes
    # ================================================================
    def update(self):
        """Retourne (linear_x, angular_z, tir)"""

        if not self.connected or self.joystick is None:
            return 0, 0, False

        self.tir_demande = False

        # --- Lecture Entrées (Raw Input) ---
        pygame.event.pump()
        axis_x = self.joystick.get_axis(self.AXIS_LEFT_X)  # gauche/droite
        axis_y = self.joystick.get_axis(self.AXIS_LEFT_Y)  # avant/arrière

        # Deadzone
        axis_x = self._apply_deadzone(axis_x)
        axis_y = self._apply_deadzone(axis_y)
        
        # --- Calcul Cibles (Target Velocities) ---
        # Stick PS3 : vers le haut = valeur négative
        target_linear = -axis_y * self.vitesse_max_lineaire
        
        # Correction du bug de rotation inversee
        direction_factor = -1 if target_linear < 0 else 1
        target_angular = -axis_x * self.vitesse_max_angulaire * direction_factor

        # D-pad override (optionnel)
        try:
            if self.joystick.get_numhats() > 0:
                hat_x, hat_y = self.joystick.get_hat(0)
                if hat_y != 0: target_linear = hat_y * self.vitesse_max_lineaire
                if hat_x != 0: target_angular = -hat_x * self.vitesse_max_angulaire * direction_factor
        except Exception:
            pass

        # --- Application directe (pas de rampe) ---
        # On applique la vitesse cible immédiatement
        self.current_linear = target_linear
        self.current_angular = target_angular

        # --- Clamping Final ---
        self.current_linear = max(min(self.current_linear, self.strict_max_linear), -self.strict_max_linear)
        
        msg_max_ang = abs(self.strict_max_angular)
        self.current_angular = max(min(self.current_angular, msg_max_ang), -msg_max_ang)

        # --- Conversion ROS ---
        # PLUS DE 0.1 car tout est en m/s désormais
        linear_x = self.current_linear
        angular_z = math.radians(self.current_angular)

        return linear_x, angular_z, self.tir_demande


    # ================================================================
    #   Gestion des événements
    # ================================================================
    def handle_event(self, event):

        if not self.connected:
            return False

        # Bouton X  arrêt d’urgence
        if event.type == pygame.JOYBUTTONDOWN:
            if event.button == self.BTN_X:
                self.tir_demande = True
                print(" PS3: Bouton X  ARRÊT D’URGENCE")
                return True

        # Deconnexion
        if event.type == pygame.JOYDEVICEREMOVED:
            print("[PS3] Manette deconnectee")
            self.connected = False
            self.joystick = None

        # Connexion d’une nouvelle manette
        if event.type == pygame.JOYDEVICEADDED:
            print("[PS3] Nouvelle manette detectee")
            self._detect_controller()

        return False


    # ================================================================
    #   Statut de connexion
    # ================================================================
    def is_connected(self):
        """Retourne True si une manette est connectée."""
        return self.connected
