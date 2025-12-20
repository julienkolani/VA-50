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
import config


class PS3Controller:
    """Contrôleur manette PS3 configurable pour TurtleBot"""

    def __init__(self, vitesse_factor=None):

        # Charger config PS3
        cfg = config.get_ps3_config()

        self.vitesse_factor = (
            vitesse_factor if vitesse_factor is not None
            else config.INITIAL_SPEED_FACTOR
        )

        self.joystick = None
        self.connected = False
        self.tir_demande = False

        # Deadzone depuis config
        self.deadzone = cfg["deadzone"]

        # Vitesse max depuis config (scalée par vitesse_factor)
        self.vitesse_max_lineaire = cfg["max_linear"] * self.vitesse_factor
        self.vitesse_max_angulaire = cfg["max_angular"] * self.vitesse_factor

        # Mapping boutons (configurable)
        self.BTN_X = cfg["btn_x"]
        self.BTN_CIRCLE = cfg["btn_circle"]
        self.BTN_TRIANGLE = cfg["btn_triangle"]
        self.BTN_SQUARE = cfg["btn_square"]

        # Mapping axes
        self.AXIS_LEFT_X = cfg["axis_left_x"]
        self.AXIS_LEFT_Y = cfg["axis_left_y"]
        self.AXIS_RIGHT_X = cfg["axis_right_x"]

        # Init manette pygame
        pygame.joystick.init()
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
    def set_vitesse_factor(self, factor):

        ratio = factor / self.vitesse_factor
        self.vitesse_factor = factor

        cfg = config.get_ps3_config()

        self.vitesse_max_lineaire = cfg["max_linear"] * factor
        self.vitesse_max_angulaire = cfg["max_angular"] * factor


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
    def update(self):
        """Retourne (linear_x, angular_z, tir)"""

        if not self.connected or self.joystick is None:
            return 0, 0, False

        self.tir_demande = False

        # ----------------------------------------------------------
        #  Lecture des axes (stick gauche)
        # ----------------------------------------------------------
        axis_x = self.joystick.get_axis(self.AXIS_LEFT_X)  # gauche/droite
        axis_y = self.joystick.get_axis(self.AXIS_LEFT_Y)  # avant/arrière

        # Deadzone
        axis_x = self._apply_deadzone(axis_x)
        axis_y = self._apply_deadzone(axis_y)

        # Stick PS3 : vers le haut = valeur négative
        vitesse_lineaire = -axis_y * self.vitesse_max_lineaire

        # Correction du bug de rotation inversee (meme logique que clavier)
        direction_factor = -1 if vitesse_lineaire < 0 else 1

        vitesse_angulaire = -axis_x * self.vitesse_max_angulaire * direction_factor

        # ----------------------------------------------------------
        #  D-pad override (haut / bas / gauche / droite)
        # ----------------------------------------------------------
        try:
            if self.joystick.get_numhats() > 0:
                hat_x, hat_y = self.joystick.get_hat(0)

                # Avant / arrière
                if hat_y != 0:
                    vitesse_lineaire = hat_y * self.vitesse_max_lineaire

                # Gauche / droite
                if hat_x != 0:
                    vitesse_angulaire = -hat_x * self.vitesse_max_angulaire * direction_factor

        except Exception:
            pass

        # ----------------------------------------------------------
        # Conversion ROS
        # ----------------------------------------------------------
        linear_x = vitesse_lineaire * 0.1
        angular_z = math.radians(vitesse_angulaire)

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
