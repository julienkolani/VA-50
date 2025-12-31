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
    def __init__(self, config):
        """
        Initialise l'interface intégrée.
        Args:
            config (dict): Dictionnaire contenant toute la configuration
                           (ui, network, robot, controls)
        """
        pygame.init()
        self.config = config
        
        # Sections spécifiques
        ui_cfg = config.get('ui', {})
        network_cfg = config.get('network', {})
        robot_cfg = config.get('robot', {})
        controls_cfg = config.get('controls', {})

        # 1. Config Fenêtre / UI
        win_cfg = ui_cfg.get('window', {})
        panel_cfg = ui_cfg.get('panels', {})
        
        self.width = win_cfg.get('default_width', 1200)
        self.height = win_cfg.get('default_height', 700)
        self.info_width = panel_cfg.get('info_width', 350) 
        self.target_fps = win_cfg.get('target_fps', 60)
        self.status_update_interval = panel_cfg.get('status_update_interval_s', 3.0)

        self.screen = pygame.display.set_mode(
            (self.width, self.height), pygame.RESIZABLE
        )
        pygame.display.set_caption(win_cfg.get('title', "TurtleBot Controller"))

        self.clock = pygame.time.Clock()

        # Thème (couleurs + polices)
        self.theme = ui_cfg.get('theme', {})
        self.colors = self.theme.get('colors', {}) # Faudra gérer les defaults si vide
        self.fonts_cfg = ui_cfg.get('fonts', {})
        self.load_theme_fonts() # Refactor needed here if relying on defaults

        # 2. Config WebSocket
        ws_cfg_section = network_cfg.get('websocket', {})
        self.ws_uri = ws_cfg_section.get('uri', "ws://localhost:8765")
        
        # Injecte la config réseau complète (pour K factors)
        self.ws_client = WebSocketClient(self.ws_uri, config=network_cfg)

        # 3. Robot Visuel
        # Layout initial
        self.update_layout(self.width, self.height)
        cx = self.sim_rect.centerx
        cy = self.sim_rect.centery
        
        visual_cfg = ui_cfg.get('visual', {})
        self.visual_robot = VisualRobot(cx, cy, 0, config=visual_cfg)

        # 4. Contrôleurs (Keyboard / PS3)
        # On passe la section 'controls' mais aussi 'robot' pour les limites physiques
        
        # Construit un config object fusionné ou passe tout
        controller_init_config = {
            'controls': controls_cfg,
            'robot': robot_cfg
        }
        
        self.keyboard_controller = KeyboardController(config=controller_init_config)
        self.ps3_controller = PS3Controller(config=controller_init_config)

        # Auto-detection manette
        if self.ps3_controller.is_connected():
            self.control_mode = "ps3"
            logger.info("Mode initial: Manette PS3")
        else:
            self.control_mode = "keyboard"
            # logger.info("Mode initial: Clavier")

        self.running = True
        self.last_status_request = 0




    def load_theme_fonts(self):
        """Charge les polices et couleurs depuis la config."""
        # Couleurs (fallback si config vide)
        self.bg_color = tuple(self.colors.get('background', (15, 15, 20)))
        self.panel_color = tuple(self.colors.get('panel', (25, 28, 35)))
        self.panel_accent = tuple(self.colors.get('panel_accent', (35, 40, 50)))
        
        self.text_color = tuple(self.colors.get('text', (230, 230, 235)))
        self.text_dim = tuple(self.colors.get('text_dim', (150, 150, 160)))
        self.text_bright = tuple(self.colors.get('text_bright', (255, 255, 255)))
        
        self.accent_color = tuple(self.colors.get('accent', (0, 230, 255)))
        self.accent_bright = tuple(self.colors.get('accent_bright', (100, 240, 255)))
        self.accent_dim = tuple(self.colors.get('accent_dim', (0, 180, 200)))
        
        self.success_color = tuple(self.colors.get('success', (50, 255, 150)))
        self.warning_color = tuple(self.colors.get('warning', (255, 180, 0)))
        self.error_color = tuple(self.colors.get('error', (255, 80, 100)))

        # Polices
        sizes = self.fonts_cfg
        try:
            self.font_title = pygame.font.Font(None, sizes.get('title_size', 36))
            self.font_sub = pygame.font.Font(None, sizes.get('subtitle_size', 28))
            self.font_normal = pygame.font.Font(None, sizes.get('normal_size', 24))
            self.font_small = pygame.font.Font(None, sizes.get('small_size', 20))
            self.font_tiny = pygame.font.Font(None, sizes.get('tiny_size', 16))
        except Exception as e:
            # logger.error(f"Erreur chargement polices: {e}")
            self.font_normal = pygame.font.Font(None, 24)

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
        # Utilisation d'une valeur fixe pour l'ombre (0,0,0,80) car config globale supprimée
        pygame.draw.rect(shadow, (0, 0, 0, 80), shadow.get_rect(), border_radius=12)
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
        pulse = (math.sin(time.time() * 2.0) + 1) / 2 # 2.0 = pulse speed default
        factor = 0.75 + (1.25 - 0.75) * pulse # lerp manual impl
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
        surf = self.font_title.render(f"MICRO-SIMULATION {mode_icon}", True, self.accent_bright)
        rect = surf.get_rect(center=(cx, self.sim_rect.y + 40))
        self.screen.blit(surf, rect)

        # Grille
        spacing = 50
        gc = self.colors.get('grid', (35, 38, 45))
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
                surf = self.font_title.render("EN ATTENTE", True, self.text_dim)
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
        cmd_freq = 20.0
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
                        # Increments = 0.1, Max = 2.0
                        new = min(2.0,
                                  self.keyboard_controller.vitesse_factor + 0.1)
                        self.keyboard_controller.set_vitesse_factor(new)
                        self.ps3_controller.set_vitesse_factor(new)

                    if event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                        # Decrements = 0.1, Min = 0.1
                        new = max(0.1,
                                  self.keyboard_controller.vitesse_factor - 0.1)
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
            if now - self.last_status_request >= self.status_update_interval:
                self.ws_client.request_status()
                self.last_status_request = now

            # ------------------- RENDU -------------------
            self.screen.fill(self.bg_color)

            stats = self.ws_client.get_stats()
            self.draw_stats_panel(stats)
            self.draw_simulation_panel(linear_x, angular_z)

            pygame.display.flip()
            self.clock.tick(self.target_fps)

        pygame.quit()
