#!/usr/bin/env python3
"""
Module Keyboard Controller
"""

import pygame
import math



class KeyboardController:
    """Contrôleur clavier configurable pour TurtleBot"""

    def __init__(self, config):
        """
        Initialise le contrôleur clavier.
        Args:
            config (dict): Configuration globale (doit contenir 'controls' et 'robot')
        """
        ctrl_cfg = config.get('controls', {})
        kb_cfg = ctrl_cfg.get('keyboard', {})
        speed_cfg = ctrl_cfg.get('speed_control', {})
        
        # Facteur de vitesse initial
        self.vitesse_factor = speed_cfg.get('initial_factor', 1.0)

        # États internes
        self.vitesse_lineaire = 0.0
        self.vitesse_angulaire = 0.0
        self.tir_demande = False

        # Paramètres (accel, friction)
        self.accel_lineaire = kb_cfg.get('accel_linear', 0.2) * self.vitesse_factor
        self.accel_angulaire = kb_cfg.get('accel_angular', 2.5) * self.vitesse_factor
        self.friction = kb_cfg.get('friction_linear', 0.92)
        self.friction_angulaire = kb_cfg.get('friction_angular', 0.82)
        
        # Rampes de lissage
        self.ramp_linear = speed_cfg.get('acceleration_ramp_linear', 0.01)
        self.ramp_angular = speed_cfg.get('acceleration_ramp_angular', 0.05)
        
        # -- Définition des VITESSES MAX --
        # La config robot contient directement velocity_limits (pas imbriqué)
        robot_cfg = config.get('robot', {})
        robot_limits = robot_cfg.get('velocity_limits', {})
        
        # 1. Limite Absolue (Physique) en m/s et rad/s
        self.limit_linear_mps = robot_limits.get('max_linear_mps', 0.22)
        self.limit_angular_radps = robot_limits.get('max_angular_radps', 4.2)
        
        # 2. Vitesse Max Contrôleur (pour mapping Input -> Vitesse)
        # On utilise directement la limite physique comme max atteignable à 100% de facteur
        # (L'ancien code utilisait une valeur "interne" x10. On passe en SI : m/s et deg/s)
        
        # NOTE IMPORTANTE: Le code update() attend des degrés pour l'angulaire
        # On va continuer à travailler en DEGRES pour l'angulaire interne pour l'instant
        # pour minimiser les changements de logique, mais on clamp strict à la fin.
        
        self.vitesse_max_lineaire = self.limit_linear_mps * self.vitesse_factor
        self.vitesse_max_angulaire = math.degrees(self.limit_angular_radps) * self.vitesse_factor
        
        # Limites strictes (constantes)
        self.strict_max_linear = self.limit_linear_mps
        self.strict_max_angular = math.degrees(self.limit_angular_radps)

    # ============================================================
    #   MISE À JOUR DU FACTEUR DE VITESSE (+ et -)
    # ============================================================
    # ============================================================
    #   MISE À JOUR DU FACTEUR DE VITESSE (+ et -)
    # ============================================================
    def set_vitesse_factor(self, factor):

        ratio = factor / self.vitesse_factor
        self.vitesse_factor = factor

        # Recalcul des vitesses max basées sur les limites physiques
        self.vitesse_max_lineaire = self.limit_linear_mps * self.vitesse_factor
        self.vitesse_max_angulaire = math.degrees(self.limit_angular_radps) * self.vitesse_factor

        # Ajustement des vitesses actuelles
        self.vitesse_lineaire *= ratio
        self.vitesse_angulaire *= ratio

    # ============================================================
    #   MISE À JOUR DES MOUVEMENTS
    # ============================================================
    def update(self, touches):
        self.tir_demande = False

        # --- Cibles de vitesse (Target Velocity) ---
        target_linear = 0.0
        target_angular = 0.0

        # Linéaire
        if touches[pygame.K_UP] or touches[pygame.K_w]:
            target_linear = self.vitesse_max_lineaire  # Déjà en m/s (si config correcte)
        elif touches[pygame.K_DOWN] or touches[pygame.K_s]:
            target_linear = -self.vitesse_max_lineaire

        # Angulaire
        gauche = touches[pygame.K_LEFT] or touches[pygame.K_a]
        droite = touches[pygame.K_RIGHT] or touches[pygame.K_d]

        # Correction : on tourne plus vite si on avance pas (pivot)
        max_ang = self.vitesse_max_angulaire
        if gauche:
            target_angular = -max_ang  # Left = negative (clockwise from top view)
        elif droite:
            target_angular = max_ang   # Right = positive (counter-clockwise)

        # --- Application directe (pas de rampe) ---
        # On applique la vitesse cible immédiatement
        self.vitesse_lineaire = target_linear
        self.vitesse_angulaire = target_angular

        # --- Clamping Final (Sécurité) ---
        # On s'assure de ne jamais dépasser les limites configurées
        self.vitesse_lineaire = max(min(self.vitesse_lineaire, self.strict_max_linear), -self.strict_max_linear)
        
        msg_max_ang = abs(self.strict_max_angular)
        self.vitesse_angulaire = max(min(self.vitesse_angulaire, msg_max_ang), -msg_max_ang)

        # --- Conversion pour ROS (Output) ---
        # MAINTENANT EN SI (m/s) : Pas de multiplication par 0.1 !
        # La logique interne: self.vitesse_lineaire est bornée par self.limit_linear_mps (ex: 0.22)
        # Donc c'est DIRECTEMENT la valeur à envoyer.
        
        linear_x = self.vitesse_lineaire
        angular_z = math.radians(self.vitesse_angulaire)

        return linear_x, angular_z, self.tir_demande

    # ============================================================
    #   ÉVÉNEMENTS (ARRÊT D'URGENCE)
    # ============================================================
    def handle_event(self, event):
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                self.tir_demande = True
                return True
        return False
