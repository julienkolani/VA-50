#!/usr/bin/env python3
"""
Interface graphique integree pour TurtleBot Controller.

Ce module fournit une interface utilisateur moderne basee sur Pygame
pour controler le TurtleBot. Il inclut un panneau de simulation visuelle,
un panneau de statistiques, et supporte les controleurs clavier et PS3.

Classes:
    IntegratedUI: Interface graphique principale avec simulation et stats.
"""

import pygame
import math
import time

import config  # Chargement du thème
from websocket_client import WebSocketClient
from keyboard_controller import KeyboardController
from ps3_controller import PS3Controller
from visual_robot import VisualRobot


# ============================================================
# Outils internes : interpolation / pulse
# ============================================================

def lerp(a, b, t):
    """Interpolation linéaire"""
    return a + (b - a) * t


# ============================================================
#                   CLASS INTEGRATED UI
# ============================================================

class IntegratedUI:
    """Interface graphique TurtleBot moderne (Pygame)"""

    # -----------------------------------------------------
    #  INITIALISATION
    # -----------------------------------------------------
    def __init__(self, width=None, height=None, ws_uri=None):
        pygame.init()

        ui_cfg = config.get_ui_config()

        # Dimensions : priorité aux paramètres envoyés à __init__
        self.width = width if width is not None else ui_cfg['default_width']
        self.height = height if height is not None else ui_cfg['default_height']
        self.info_width = ui_cfg['info_panel_width']

        # WebSocket URI : priorité au paramètre constructor
        self.ws_uri = ws_uri if ws_uri is not None else config.WEBSOCKET_URI

        self.screen = pygame.display.set_mode(
            (self.width, self.height), pygame.RESIZABLE
        )
        pygame.display.set_caption("TurtleBot Control - UI")

        self.clock = pygame.time.Clock()

        # Thème moderne (couleurs + polices)
        self.load_theme()

        # WebSocket client
        self.ws_client = WebSocketClient(self.ws_uri)

        # Layout
        self.update_layout(self.width, self.height)

        # Robot visuel
        cx = self.sim_rect.centerx
        cy = self.sim_rect.centery
        self.visual_robot = VisualRobot(cx, cy, 0)

        # Contrôleurs
        ctrl_cfg = config.get_controller_config()
        self.keyboard_controller = KeyboardController(
            vitesse_factor=ctrl_cfg["initial_speed_factor"]
        )
        self.ps3_controller = PS3Controller(
            vitesse_factor=ctrl_cfg["initial_speed_factor"]
        )

        # Auto-detection manette
        self.control_mode = "ps3" if self.ps3_controller.is_connected() else "keyboard"

        self.last_status_request = 0


    # -----------------------------------------------------
    #  CHARGEMENT DU THÈME (couleurs + polices)
    # -----------------------------------------------------
    def load_theme(self):
        colors = config.get_color_scheme()

        # Couleurs
        self.bg_color = colors["bg"]
        self.panel_color = colors["panel"]
        self.panel_accent = colors["panel_accent"]
        self.text_color = colors["text"]
        self.text_dim = colors["text_dim"]
        self.text_bright = colors["text_bright"]
        self.accent_color = colors["accent"]
        self.accent_bright = colors["accent_bright"]
        self.accent_dim = colors["accent_dim"]
        self.success_color = colors["success"]
        self.warning_color = colors["warning"]
        self.error_color = colors["error"]

        # Polices Roboto modernisées
        # -----------------------------------------------------
        #  Polices (avec fallback automatique)
        # -----------------------------------------------------
        def safe_font(path, size):
            """Charge une police, ou fallback sur la police par défaut."""
            try:
                return pygame.font.Font(path, size)
            except FileNotFoundError:
                print(f"[UI] Police introuvable : {path} - utilisation de la police par defaut.")
                return pygame.font.Font(None, size)

        # Polices Roboto (si absentes -> police par defaut)
        self.font_titre = safe_font("assets/Roboto-Bold.ttf", config.FONT_SIZE_TITLE)
        self.font_sub = safe_font("assets/Roboto-Medium.ttf", config.FONT_SIZE_SUBTITLE)
        self.font_normal = safe_font("assets/Roboto-Regular.ttf", config.FONT_SIZE_NORMAL)
        self.font_small = safe_font("assets/Roboto-Light.ttf", config.FONT_SIZE_SMALL)
        self.font_tiny = safe_font("assets/Roboto-Light.ttf", config.FONT_SIZE_TINY)

    # -----------------------------------------------------
    #  MISE EN PAGE
    # -----------------------------------------------------
    def update_layout(self, width, height):
        self.width = width
        self.height = height

        self.sim_rect = pygame.Rect(
            10, 10, width - self.info_width - 30, height - 20
        )
        self.info_rect = pygame.Rect(
            width - self.info_width, 10, self.info_width - 10, height - 20
        )

        # Recentre le robot si déjà présent
        if hasattr(self, "visual_robot"):
            self.visual_robot.rest_x = self.sim_rect.centerx
            self.visual_robot.rest_y = self.sim_rect.centery
            self.visual_robot.reset_position()

    # -----------------------------------------------------
    #  EFFETS MODERNES : ombres, sous-panneaux, titres, pulse
    # -----------------------------------------------------

    def draw_shadow(self, rect, size=12):
        """Ombre portée moderne"""
        shadow = pygame.Surface((rect.width, rect.height), pygame.SRCALPHA)
        pygame.draw.rect(shadow, config.COLOR_SHADOW, shadow.get_rect(), border_radius=12)
        self.screen.blit(shadow, (rect.x + size, rect.y + size))

    def draw_subpanel(self, x, y, w, h, radius=12):
        """Sous-panneau semi-transparent"""
        surf = pygame.Surface((w, h), pygame.SRCALPHA)
        pygame.draw.rect(surf, (self.panel_accent[0], self.panel_accent[1], self.panel_accent[2], 90),
                         (0, 0, w, h), border_radius=radius)
        self.screen.blit(surf, (x, y))
        return pygame.Rect(x, y, w, h)

    def draw_title(self, text, x, y, color=None):
        """Titre moderne + ligne accent"""
        if color is None:
            color = self.accent_color

        surf = self.font_sub.render(text, True, color)
        self.screen.blit(surf, (x, y))

        pygame.draw.line(self.screen, color, (x, y + 32), (x + 220, y + 32), 3)

        return y + 50

    def pulse_color(self, base_color):
        """Couleur pulsante douce pour l'État connexion, FPS…"""
        pulse = (math.sin(time.time() * config.ANIMATION_PULSE_SPEED) + 1) / 2
        factor = lerp(0.75, 1.25, pulse)
        return tuple(min(int(c * factor), 255) for c in base_color)

    # -----------------------------------------------------
    #  PANNEAU STATS (VERSION MODERNE)
    # -----------------------------------------------------
    def draw_stats_panel(self, stats):

        # Ombre + panneau principal
        self.draw_shadow(self.info_rect)
        pygame.draw.rect(self.screen, self.panel_color, self.info_rect, border_radius=14)

        x = self.info_rect.x + 20
        y = self.info_rect.y + 20

        # --- Section ROS BRIDGE ---
        y = self.draw_title("ROS BRIDGE", x, y)

        sub = self.draw_subpanel(x, y, self.info_rect.width - 40, 110)
        y += 20

        # Connexion animée
        ccol = self.pulse_color(self.success_color) if stats["connected"] else self.error_color
        pygame.draw.circle(self.screen, ccol, (x + 20, y + 12), 10)

        label = "CONNECTÉ" if stats["connected"] else "DÉCONNECTÉ"
        self.screen.blit(self.font_normal.render(label, True, ccol), (x + 45, y))

        y += 40

        # Mode de controle
        mode = "Manette PS3" if self.control_mode == "ps3" else "Clavier"
        mcol = self.accent_bright if self.control_mode == "ps3" else self.text_color

        self.screen.blit(
            self.font_small.render("Mode : " + mode, True, mcol),
            (x + 20, y)
        )

        y += 70

        # --- Section statistiques ---
        y = self.draw_title("STATISTIQUES", x, y)

        sub = self.draw_subpanel(x, y, self.info_rect.width - 40, 160)
        y += 20

        stats_items = [
            ("Envoyées", stats["sent"], self.text_color),
            ("Acceptées", stats["accepted"], self.success_color),
            ("Rejetées", stats["rejected"],
             self.error_color if stats["rejected"] else self.text_dim),
            ("Succès", f"{stats['success_rate']*100:.1f}%",
             self.success_color),
        ]

        for label, value, col in stats_items:
            self.screen.blit(self.font_small.render(f"{label} :", True, self.text_dim), (x + 20, y))
            self.screen.blit(self.font_small.render(str(value), True, col), (x + 160, y))
            y += 28

        y += 20

        # --- Section sécurité ---
        y = self.draw_title("SÉCURITÉ", x, y)

        sub = self.draw_subpanel(x, y, self.info_rect.width - 40, 100)
        y += 20

        safety = stats.get("safety_info", {})
        if safety.get("current_pose"):
            px, py, pa = safety["current_pose"]
            self.screen.blit(self.font_small.render(f"Pos: ({px:.2f}, {py:.2f})",
                                                    True, self.text_color),
                             (x + 20, y))
            y += 25
            self.screen.blit(
                self.font_small.render(f"Angle: {math.degrees(pa):.1f}°",
                                       True, self.text_color),
                (x + 20, y)
            )

        # --- FPS ---
        sub = self.draw_subpanel(x, self.info_rect.bottom - 140,
                                 self.info_rect.width - 40, 120)

        fps = self.clock.get_fps()
        fps_color = self.pulse_color(
            self.success_color if fps > 55 else
            self.warning_color if fps > 30 else
            self.error_color
        )

        self.screen.blit(self.font_sub.render(f"{fps:.1f} FPS", True, fps_color),
                         (x + 30, self.info_rect.bottom - 110))

    # -----------------------------------------------------
    #  PANNEAU SIMULATION (inchangé + couleurs modernisées)
    # -----------------------------------------------------
    def draw_simulation_panel(self, linear_x, angular_z):
        pygame.draw.rect(self.screen, self.panel_color, self.sim_rect, border_radius=14)
        pygame.draw.rect(self.screen, self.accent_dim, self.sim_rect, 2, border_radius=14)

        cx, cy = self.sim_rect.center

        # Titre
        mode_icon = "[PS3]" if self.control_mode == "ps3" else "[KB]"
        surf = self.font_titre.render(f"MICRO-SIMULATION {mode_icon}", True, self.accent_bright)
        rect = surf.get_rect(center=(cx, self.sim_rect.y + 40))
        self.screen.blit(surf, rect)

        # Grille
        spacing = 50
        gc = config.COLOR_GRID
        for i in range(self.sim_rect.left + 20, self.sim_rect.right - 20, spacing):
            pygame.draw.line(self.screen, gc,
                             (i, self.sim_rect.top + 80),
                             (i, self.sim_rect.bottom - 80), 1)

        for i in range(self.sim_rect.top + 80, self.sim_rect.bottom - 80, spacing):
            pygame.draw.line(self.screen, gc,
                             (self.sim_rect.left + 20, i),
                             (self.sim_rect.right - 20, i), 1)

        # Robot
        self.visual_robot.draw(self.screen, scale=1.0)

        # Messages de vitesse
        info_y = self.sim_rect.y + 80

        if abs(linear_x) > 0.01:
            col = self.success_color if linear_x > 0 else self.warning_color
            direction = "Avant" if linear_x > 0 else "Arrière"
            msg = f"{direction}: {abs(linear_x):.3f} m/s"
            surf = self.font_normal.render(msg, True, col)
            self.screen.blit(surf, surf.get_rect(center=(cx, info_y)))

        if abs(angular_z) > 0.01:
            col = self.accent_color
            direction = "Gauche" if angular_z > 0 else "Droite"
            msg = f"Rotation {direction}: {abs(angular_z):.3f} rad/s"
            surf = self.font_normal.render(msg, True, col)
            self.screen.blit(surf,
                             surf.get_rect(center=(cx, self.sim_rect.bottom - 60)))

        # Mode inactif
        if abs(linear_x) < 0.01 and abs(angular_z) < 0.01:
            dist = math.dist((self.visual_robot.x, self.visual_robot.y),
                             (self.visual_robot.rest_x, self.visual_robot.rest_y))

            if dist < 2:
                surf = self.font_titre.render("EN ATTENTE", True, self.text_dim)
                self.screen.blit(surf, surf.get_rect(center=(cx, cy - 60)))

                hint = "Flèches / WASD" if self.control_mode == "keyboard" else "Stick gauche"
                surf = self.font_small.render(hint, True, self.text_dim)
                self.screen.blit(surf, surf.get_rect(center=(cx, cy + 80)))
            else:
                surf = self.font_small.render("Retour au centre…", True, self.warning_color)
                self.screen.blit(surf, surf.get_rect(center=(cx, cy - 80)))

    # -----------------------------------------------------
    #  BOUCLE PRINCIPALE
    # -----------------------------------------------------
    def run(self):

        # Connexion WebSocket
        if not self.ws_client.start():
            print("[UI] Impossible de se connecter au ROS Bridge.")
            return

        running = True
        cmd_freq = config.COMMAND_FREQUENCY
        cmd_interval = 1.0 / cmd_freq
        last_cmd_time = time.time()

        while running:

            dt = self.clock.get_time() / 1000.0

            # ---------------- ÉVÉNEMENTS ----------------
            for event in pygame.event.get():

                if event.type == pygame.QUIT:
                    running = False

                elif event.type == pygame.VIDEORESIZE:
                    self.update_layout(event.w, event.h)

                elif event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_ESCAPE, pygame.K_q):
                        running = False

                    elif event.key == pygame.K_m:
                        if self.control_mode == "keyboard" and self.ps3_controller.is_connected():
                            self.control_mode = "ps3"
                        else:
                            self.control_mode = "keyboard"

                    elif event.key == pygame.K_c:
                        self.visual_robot.reset_trail()

                    # Vitesse
                    if event.key in (pygame.K_EQUALS, pygame.K_KP_PLUS):
                        new = min(config.MAX_SPEED_FACTOR,
                                  self.keyboard_controller.vitesse_factor + config.SPEED_INCREMENT)
                        self.keyboard_controller.set_vitesse_factor(new)
                        self.ps3_controller.set_vitesse_factor(new)

                    if event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                        new = max(config.MIN_SPEED_FACTOR,
                                  self.keyboard_controller.vitesse_factor - config.SPEED_INCREMENT)
                        self.keyboard_controller.set_vitesse_factor(new)
                        self.ps3_controller.set_vitesse_factor(new)

            # ------------ MISE À JOUR CONTRÔLES ----------
            if self.control_mode == "keyboard":
                keys = pygame.key.get_pressed()
                linear_x, angular_z, _ = self.keyboard_controller.update(keys)
            else:
                linear_x, angular_z, _ = self.ps3_controller.update()

            # Simulation robot
            self.visual_robot.update(linear_x, angular_z, dt)

            # Envoi cmd_vel à 30 Hz
            now = time.time()
            if now - last_cmd_time >= cmd_interval:
                if self.ws_client.connected:
                    self.ws_client.send_cmd_vel(linear_x, angular_z)
                last_cmd_time = now

            # Status régulier
            if now - self.last_status_request >= config.STATUS_UPDATE_INTERVAL:
                self.ws_client.request_status()
                self.last_status_request = now

            # ------------------- RENDU -------------------
            self.screen.fill(self.bg_color)

            stats = self.ws_client.get_stats()
            self.draw_stats_panel(stats)
            self.draw_simulation_panel(linear_x, angular_z)

            pygame.display.flip()
            self.clock.tick(config.TARGET_FPS)

        pygame.quit()
