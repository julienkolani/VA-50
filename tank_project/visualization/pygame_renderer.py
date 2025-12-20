"""
Moteur de Rendu Pygame - Visualisation de Jeu Moderne

Rendu premium avec :
- Placement stratégique du HUD (coins, n'obscurcit jamais les ArUcos au centre)
- Écrans de début/fin de match avec animations
- Visualisation du chemin de débogage (bascule avec la touche 'D')
- Score dynamique avec retour visuel
- Design moderne (dégradés, ombres, polices premium)

Logs : préfixe [VIS]
"""

import pygame
import numpy as np
import os
import math
from typing import Dict, Tuple, Optional, List
from datetime import datetime


class PygameRenderer:
    """
    Moteur de rendu de jeu moderne avec design HUD tactique.
    
    Fonctionnalités :
    - HUD basé sur les coins (n'obstrue pas les ArUcos centraux)
    - États du match (attente, compte à rebours, jeu, terminé)
    - Visualisation chemin de débogage (bascule D)
    - Animations de score et effets visuels
    """
    
    # États du match
    STATE_WAITING = "waiting"
    STATE_COUNTDOWN = "countdown"
    STATE_PLAYING = "playing"
    STATE_FINISHED = "finished"
    
    def __init__(self, width: int = 1024, height: int = 768, margin: int = 50, 
                 fullscreen: bool = True, display_index: int = 0):
        """
        Initialise le rendu avec une UI de jeu moderne.
        
        Args:
            width: Largeur projecteur
            height: Hauteur projecteur
            margin: Marge zone sûre
            fullscreen: Si True, mode plein écran
            display_index: 0 = principal, 1 = projecteur
        """
        # Définit la position d'affichage AVANT pygame.init()
        if display_index == 1:
            os.environ['SDL_VIDEO_WINDOW_POS'] = f'{width},0'
        elif display_index == 0:
            os.environ['SDL_VIDEO_WINDOW_POS'] = '0,0'
        
        pygame.init()
        
        self.base_width = width
        self.base_height = height
        self.width = width
        self.height = height
        self.margin = margin
        self.fullscreen = fullscreen
        
        self.draw_width = width - 2 * margin
        self.draw_height = height - 2 * margin
        
        # Crée l'affichage - démarre toujours en mode fenêtré redimensionnable pour la stabilité
        self._create_display()
        
        pygame.display.set_caption("Tank Arena - Combat Tactique")
        
        # Polices - Hiérarchie premium
        pygame.font.init()
        self.font_title = pygame.font.SysFont('Arial Black', 72, bold=True)
        self.font_large = pygame.font.SysFont('Arial Black', 48, bold=True)
        self.font_medium = pygame.font.SysFont('Arial', 36, bold=True)
        self.font_small = pygame.font.SysFont('Arial', 24)
        self.font_tiny = pygame.font.SysFont('Courier', 16)
        
        # Palette de couleurs premium
        self.BG_DARK = (20, 20, 25)
        self.BG_LIGHT = (240, 240, 245)
        
        self.AI_PRIMARY = (50, 150, 255)      # Blue
        self.AI_GLOW = (100, 200, 255)
        self.AI_DARK = (20, 80, 150)
        
        self.HUMAN_PRIMARY = (255, 80, 80)    # Red
        self.HUMAN_GLOW = (255, 150, 150)
        self.HUMAN_DARK = (180, 30, 30)
        
        self.ACCENT_GOLD = (255, 200, 50)
        self.ACCENT_GREEN = (50, 255, 150)
        self.ACCENT_ORANGE = (255, 150, 50)
        
        self.TEXT_DARK = (30, 30, 35)
        self.TEXT_LIGHT = (250, 250, 255)
        self.TEXT_GRAY = (150, 150, 160)
        
        self.OVERLAY_DARK = (0, 0, 0, 180)    # Semi-transparent
        self.OVERLAY_LIGHT = (255, 255, 255, 200)
        
        # Arena dimensions
        self.arena_width_m = 3.0
        self.arena_height_m = 2.0
        self.scale = None
        
        # Match state
        self.match_state = self.STATE_WAITING
        self.countdown_start = None
        self.match_start_time = None
        self.winner = None
        
        # Debug visualization
        self.show_debug_path = False
        self.ai_path = []  # List of (x, y) waypoints
        
        # Visual effects
        self.score_flash_ai = 0
        self.score_flash_human = 0
        self.last_ai_score = 0
        
    def _create_display(self):
        """Create or recreate the display with current settings."""
        try:
            if self.fullscreen:
                # Try fullscreen without HWSURFACE (more compatible)
                self.screen = pygame.display.set_mode(
                    (self.width, self.height),
                    pygame.FULLSCREEN
                )
            else:
                # Windowed + resizable
                self.screen = pygame.display.set_mode(
                    (self.width, self.height),
                    pygame.RESIZABLE
                )
            print(f"[VIS] Affichage créé : {self.width}x{self.height} plein écran={self.fullscreen}")
        except pygame.error as e:
            print(f"[VIS] Erreur affichage : {e}, retour au mode fenêtré")
            self.fullscreen = False
            self.screen = pygame.display.set_mode(
                (self.base_width, self.base_height),
                pygame.RESIZABLE
            )
    
    def toggle_fullscreen(self):
        """Toggle between fullscreen and windowed mode."""
        self.fullscreen = not self.fullscreen
        if not self.fullscreen:
            # Restore base dimensions when exiting fullscreen
            self.width = self.base_width
            self.height = self.base_height
        else:
            # Get current display info for fullscreen
            info = pygame.display.Info()
            self.width = info.current_w
            self.height = info.current_h
        
        self._update_dimensions()
        self._create_display()
    
    def _update_dimensions(self):
        """Update drawing dimensions after resize."""
        self.draw_width = self.width - 2 * self.margin
        self.draw_height = self.height - 2 * self.margin
        
        # Recalculate scale
        if self.arena_width_m > 0 and self.arena_height_m > 0:
            scale_x = self.draw_width / self.arena_width_m
            scale_y = self.draw_height / self.arena_height_m
            self.scale = min(scale_x, scale_y)
    
    def handle_resize(self, new_width: int, new_height: int):
        """Handle window resize event."""
        self.width = new_width
        self.height = new_height
        self._update_dimensions()
        self.screen = pygame.display.set_mode((new_width, new_height), pygame.RESIZABLE)
        print(f"[VIS] Redimensionné à {new_width}x{new_height}")
        self.last_human_score = 0
        
    def set_arena_dimensions(self, width_m: float, height_m: float):
        """Set arena dimensions for coordinate conversion."""
        self.arena_width_m = width_m
        self.arena_height_m = height_m
        
        scale_x = self.draw_width / width_m
        scale_y = self.draw_height / height_m
        self.scale = min(scale_x, scale_y)
        
    def world_to_screen(self, x_m: float, y_m: float) -> Tuple[int, int]:
        """Convert world coordinates to screen pixels."""
        if self.scale is None:
            self.scale = 100
        
        px = self.margin + int(x_m * self.scale)
        py = self.margin + int((self.arena_height_m - y_m) * self.scale)
        
        return (px, py)
    
    def handle_keypress(self, event: pygame.event.Event):
        """
        Handle keyboard input.
        
        Args:
            event: Pygame key event
        """
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_d:
                # Toggle debug path visualization
                self.show_debug_path = not self.show_debug_path
                print("[VIS] Chemin debug : {}".format('ON' if self.show_debug_path else 'OFF'))
            elif event.key == pygame.K_F11:
                # Toggle fullscreen
                self.toggle_fullscreen()
            elif event.key == pygame.K_ESCAPE and self.fullscreen:
                # Exit fullscreen on ESC
                self.toggle_fullscreen()
    
    def start_match(self, countdown_seconds: float = 3.0):
        """Start match countdown."""
        self.match_state = self.STATE_COUNTDOWN
        self.countdown_start = pygame.time.get_ticks() / 1000.0
        self.countdown_duration = countdown_seconds
        
    def begin_playing(self):
        """Begin actual gameplay."""
        self.match_state = self.STATE_PLAYING
        self.match_start_time = pygame.time.get_ticks() / 1000.0
        
    def end_match(self, winner: str):
        """
        End match and show results.
        
        Args:
            winner: "AI" or "HUMAN"
        """
        self.match_state = self.STATE_FINISHED
        self.winner = winner
        
    def set_ai_path(self, waypoints: List[Tuple[float, float]]):
        """
        Set AI path for debug visualization.
        
        Args:
            waypoints: List of (x, y) positions in meters
        """
        self.ai_path = waypoints
    
    def render_frame(self, world_state: Dict, game_state: Dict):
        """
        Render complete frame based on match state.
        
        Args:
            world_state: Robot poses, obstacles, etc.
            game_state: Scores, time, status
        """
        # Update score flash effects
        ai_score = game_state.get('robot_4_hits_inflicted', 0)
        human_score = game_state.get('robot_5_hits_inflicted', 0)
        
        if ai_score > self.last_ai_score:
            self.score_flash_ai = 30  # Flash for 30 frames
            self.last_ai_score = ai_score
        
        if human_score > self.last_human_score:
            self.score_flash_human = 30
            self.last_human_score = human_score
        
        self.score_flash_ai = max(0, self.score_flash_ai - 1)
        self.score_flash_human = max(0, self.score_flash_human - 1)
        
        # Render based on state
        if self.match_state == self.STATE_WAITING:
            self._render_waiting_screen()
        elif self.match_state == self.STATE_COUNTDOWN:
            self._render_countdown(world_state, game_state)
        elif self.match_state == self.STATE_PLAYING:
            self._render_gameplay(world_state, game_state)
        elif self.match_state == self.STATE_FINISHED:
            self._render_end_screen(game_state)
        
        # Update display
        pygame.display.flip()
    
    def _render_waiting_screen(self):
        """Render waiting for match start screen."""
        self.screen.fill(self.BG_DARK)
        
        # Title
        title = self.font_title.render("TANK ARENA", True, self.ACCENT_GOLD)
        title_rect = title.get_rect(center=(self.width // 2, self.height // 2 - 100))
        self.screen.blit(title, title_rect)
        
        # Sous-titre
        subtitle = self.font_medium.render("Combat de Robots Tactique", True, self.TEXT_LIGHT)
        subtitle_rect = subtitle.get_rect(center=(self.width // 2, self.height // 2))
        self.screen.blit(subtitle, subtitle_rect)
        
        # Instruction
        pulse = int(128 + 127 * math.sin(pygame.time.get_ticks() / 500))
        instruction = self.font_small.render("Appuyez sur START pour commencer", True, (pulse, pulse, pulse))
        instr_rect = instruction.get_rect(center=(self.width // 2, self.height // 2 + 100))
        self.screen.blit(instruction, instr_rect)
    
    def _render_countdown(self, world_state: Dict, game_state: Dict):
        """Render countdown before match."""
        # Draw arena first (faded)
        self._render_gameplay(world_state, game_state, alpha=100)
        
        # Overlay countdown
        overlay = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 150))
        self.screen.blit(overlay, (0, 0))
        
        # Calculate countdown
        elapsed = pygame.time.get_ticks() / 1000.0 - self.countdown_start
        remaining = max(0, self.countdown_duration - elapsed)
        
        if remaining > 0:
            count_num = int(remaining) + 1
            scale = 1.0 + 0.3 * (1.0 - (remaining % 1.0))
            
            count_text = self.font_title.render(str(count_num), True, self.ACCENT_GOLD)
            count_text = pygame.transform.scale(
                count_text,
                (int(count_text.get_width() * scale), int(count_text.get_height() * scale))
            )
            count_rect = count_text.get_rect(center=(self.width // 2, self.height // 2))
            self.screen.blit(count_text, count_rect)
        else:
            # Switch to playing
            self.begin_playing()
    
    def _render_gameplay(self, world_state: Dict, game_state: Dict, alpha: int = 255):
        """Render main gameplay view."""
        # Background
        self.screen.fill(self.BG_LIGHT)
        
        # Arena border
        self._draw_arena_border()
        
        # Debug: AI path (if enabled)
        if self.show_debug_path and self.ai_path:
            self._draw_ai_path()
        
        # Obstacles
        self._draw_obstacles(world_state.get('occupancy_grid'))
        
        # Robots
        robot_4_pose = world_state.get('robot_4_pose', (0, 0, 0))
        robot_5_pose = world_state.get('robot_5_pose', (0, 0, 0))
        
        self._draw_robot(robot_4_pose, self.AI_PRIMARY, "IA", self.AI_GLOW)
        self._draw_robot(robot_5_pose, self.HUMAN_PRIMARY, "HUMAIN", self.HUMAN_GLOW)
        
        # Lock-on indicator
        if game_state.get('ai_has_los', False):
            self._draw_lock_on(robot_5_pose[:2])
        
        # HUD - STRATEGIC PLACEMENT (corners only!)
        self._draw_modern_hud(game_state)
        
        # Debug info (bottom right corner)
        if self.show_debug_path:
            self._draw_debug_info(game_state)
    
    def _draw_arena_border(self):
        """Draw arena boundary with modern style."""
        rect = pygame.Rect(self.margin, self.margin, self.draw_width, self.draw_height)
        
        # Outer glow
        for i in range(3):
            glow_rect = rect.inflate(i * 4, i * 4)
            alpha = 40 - i * 10
            s = pygame.Surface((glow_rect.width, glow_rect.height), pygame.SRCALPHA)
            pygame.draw.rect(s, (*self.TEXT_GRAY, alpha), s.get_rect(), 2)
            self.screen.blit(s, glow_rect.topleft)
        
        # Main border
        pygame.draw.rect(self.screen, self.TEXT_DARK, rect, 4)
    
    def _draw_obstacles(self, grid):
        """Draw obstacles from occupancy grid."""
        if grid is None:
            return
            
        # Iterate through occupied cells
        # Note: inefficient for very large grids, optimization would be to use static surface
        rows, cols = grid.grid.shape
        
        # Color for obstacles
        obs_color = (60, 60, 70)
        
        for r in range(rows):
            for c in range(cols):
                if grid.grid[r, c] > 0.5:
                    # Convert grid cell to world m -> screen px
                    # Grid (r, c) -> World (x, y) center
                    x_m, y_m = grid.grid_to_world(r, c)
                    
                    # Convert to screen
                    px, py = self.world_to_screen(x_m, y_m)
                    
                    # Size of cell in pixels
                    size = int(grid.resolution * self.scale) + 1  # +1 to avoid gaps
                    
                    # Draw rect
                    rect = pygame.Rect(px - size//2, py - size//2, size, size)
                    pygame.draw.rect(self.screen, obs_color, rect)
    
    def _draw_robot(self, pose: Tuple[float, float, float], color: Tuple, 
                   label: str, glow_color: Tuple):
        """Draw robot with modern visual effects."""
        x, y, theta = pose
        px, py = self.world_to_screen(x, y)
        
        radius_px = int(0.09 * self.scale) if self.scale else 30
        
        # Glow effect
        for i in range(3):
            glow_radius = radius_px + (3 - i) * 8
            alpha = 30 + i * 10
            s = pygame.Surface((glow_radius * 2, glow_radius * 2), pygame.SRCALPHA)
            pygame.draw.circle(s, (*glow_color, alpha), (glow_radius, glow_radius), glow_radius)
            self.screen.blit(s, (px - glow_radius, py - glow_radius))
        
        # Main body
        pygame.draw.circle(self.screen, color, (px, py), radius_px)
        pygame.draw.circle(self.screen, self.TEXT_DARK, (px, py), radius_px, 3)
        
        # Orientation indicator (cannon)
        line_len = radius_px * 1.8
        end_x = px + int(line_len * np.cos(theta))
        end_y = py - int(line_len * np.sin(theta))
        
        # Cannon shadow
        pygame.draw.line(self.screen, (0, 0, 0, 100), 
                        (px + 2, py + 2), (end_x + 2, end_y + 2), 5)
        # Cannon
        pygame.draw.line(self.screen, self.TEXT_DARK, (px, py), (end_x, end_y), 5)
        pygame.draw.circle(self.screen, self.ACCENT_GOLD, (end_x, end_y), 6)
    
    def _draw_ai_path(self):
        """Draw AI pathfinding visualization (debug mode)."""
        if len(self.ai_path) < 2:
            return
        
        # Convert waypoints to screen coords
        points = [self.world_to_screen(x, y) for x, y in self.ai_path]
        
        # Draw path line
        if len(points) >= 2:
            pygame.draw.lines(self.screen, self.AI_PRIMARY, False, points, 3)
        
        # Draw waypoint markers
        for i, (px, py) in enumerate(points):
            pygame.draw.circle(self.screen, self.AI_PRIMARY, (px, py), 4)
            if i == len(points) - 1:
                # Goal marker
                pygame.draw.circle(self.screen, self.ACCENT_GREEN, (px, py), 8, 2)
    
    def _draw_lock_on(self, target_pos: Tuple[float, float]):
        """Draw lock-on indicator with pulse effect."""
        px, py = self.world_to_screen(*target_pos)
        
        t = pygame.time.get_ticks() / 1000.0
        radius = 25 + int(8 * math.sin(t * 4))
        
        # Crosshair
        pygame.draw.circle(self.screen, self.HUMAN_PRIMARY, (px, py), radius, 3)
        pygame.draw.line(self.screen, self.HUMAN_PRIMARY, 
                        (px - radius - 10, py), (px - radius - 5, py), 3)
        pygame.draw.line(self.screen, self.HUMAN_PRIMARY, 
                        (px + radius + 5, py), (px + radius + 10, py), 3)
        pygame.draw.line(self.screen, self.HUMAN_PRIMARY, 
                        (px, py - radius - 10), (px, py - radius - 5), 3)
        pygame.draw.line(self.screen, self.HUMAN_PRIMARY, 
                        (px, py + radius + 5), (px, py + radius + 10), 3)
        
        # Texte "VERROUILLÉ"
        locked_text = self.font_tiny.render("VERROUILLÉ", True, self.HUMAN_PRIMARY)
        self.screen.blit(locked_text, (px - 25, py + radius + 15))
    
    def _draw_modern_hud(self, game_state: Dict):
        """
        Draw HUD in CORNERS only (never obscures center ArUcos).
        
        Layout:
        - Top-left: AI score
        - Top-right: Human score
        - Top-center: Timer (small, minimal)
        - Bottom-left: Match info
        - Bottom-right: Debug toggle hint
        """
        # --- TOP-LEFT: AI SCORE ---
        ai_score = game_state.get('robot_4_hits_inflicted', 0)
        ai_health = game_state.get('robot_4_hits_received', 0)
        
        flash_alpha = self.score_flash_ai * 8 if self.score_flash_ai > 0 else 0
        ai_bg_color = (
            min(255, self.AI_PRIMARY[0] + flash_alpha),
            min(255, self.AI_PRIMARY[1] + flash_alpha),
            min(255, self.AI_PRIMARY[2] + flash_alpha)
        )
        
        self._draw_score_corner(30, 30, "ROBOT IA", ai_score, ai_health, 
                               ai_bg_color, self.AI_DARK, "left")
        
        # --- HAUT-DROITE : SCORE HUMAIN ---
        human_score = game_state.get('robot_5_hits_inflicted', 0)
        human_health = game_state.get('robot_5_hits_received', 0)
        
        flash_alpha = self.score_flash_human * 8 if self.score_flash_human > 0 else 0
        human_bg_color = (
            min(255, self.HUMAN_PRIMARY[0] + flash_alpha),
            min(255, self.HUMAN_PRIMARY[1] + flash_alpha),
            min(255, self.HUMAN_PRIMARY[2] + flash_alpha)
        )
        
        self._draw_score_corner(self.width - 30, 30, "HUMAIN", human_score, human_health,
                               human_bg_color, self.HUMAN_DARK, "right")
        
        # --- HAUT-CENTRE : MINUTEUR (minimal) ---
        time_remaining = game_state.get('time_remaining_s', 0)
        mins = int(time_remaining // 60)
        secs = int(time_remaining % 60)
        
        timer_text = f"{mins:02d}:{secs:02d}"
        timer_surface = self.font_medium.render(timer_text, True, self.TEXT_DARK)
        timer_rect = timer_surface.get_rect(midtop=(self.width // 2, 30))
        
        # Timer background
        bg_rect = timer_rect.inflate(40, 20)
        pygame.draw.rect(self.screen, self.TEXT_LIGHT, bg_rect, border_radius=10)
        pygame.draw.rect(self.screen, self.TEXT_GRAY, bg_rect, 2, border_radius=10)
        
        self.screen.blit(timer_surface, timer_rect)
        
        # --- BAS-GAUCHE : Info match ---
        status = game_state.get('ai_state', 'IDLE')
        info_text = f"IA : {status}"
        info_surface = self.font_small.render(info_text, True, self.TEXT_GRAY)
        self.screen.blit(info_surface, (30, self.height - 60))
        
        # --- BAS-DROITE : Indice debug ---
        if self.show_debug_path:
            debug_text = "[D] Masquer Chemin"
            debug_color = self.ACCENT_GREEN
        else:
            debug_text = "[D] Afficher Chemin IA"
            debug_color = self.TEXT_GRAY
        
        debug_surface = self.font_small.render(debug_text, True, debug_color)
        debug_rect = debug_surface.get_rect(bottomright=(self.width - 30, self.height - 30))
        self.screen.blit(debug_surface, debug_rect)
    
    def _draw_score_corner(self, x: int, y: int, label: str, score: int, health: int,
                          bg_color: Tuple, border_color: Tuple, align: str):
        """Draw score panel in corner."""
        # Panel dimensions
        panel_width = 240
        panel_height = 120
        
        # Calculate position based on alignment
        if align == "right":
            panel_x = x - panel_width
        else:
            panel_x = x
        
        panel_rect = pygame.Rect(panel_x, y, panel_width, panel_height)
        
        # Background with transparency
        s = pygame.Surface((panel_width, panel_height), pygame.SRCALPHA)
        pygame.draw.rect(s, (*bg_color, 200), s.get_rect(), border_radius=15)
        pygame.draw.rect(s, border_color, s.get_rect(), 3, border_radius=15)
        self.screen.blit(s, panel_rect.topleft)
        
        # Label
        label_surface = self.font_small.render(label, True, self.TEXT_LIGHT)
        self.screen.blit(label_surface, (panel_x + 15, y + 10))
        
        # Score (large)
        score_surface = self.font_large.render(str(score), True, self.TEXT_LIGHT)
        self.screen.blit(score_surface, (panel_x + 15, y + 40))
        
        # Coups reçus
        health_text = f"Coups : {health}"
        health_surface = self.font_tiny.render(health_text, True, self.TEXT_LIGHT)
        self.screen.blit(health_surface, (panel_x + 15, y + 95))
    
    def _draw_debug_info(self, game_state: Dict):
        """Draw debug information (bottom-right, small)."""
        debug_lines = [
            f"Path waypoints: {len(self.ai_path)}",
            f"AI LOS: {game_state.get('ai_has_los', False)}",
            f"Fire request: {game_state.get('ai_fire_request', False)}"
        ]
        
        y_offset = self.height - 120
        for line in debug_lines:
            text = self.font_tiny.render(line, True, self.TEXT_GRAY)
            self.screen.blit(text, (self.width - 300, y_offset))
            y_offset += 20
    
    def _render_end_screen(self, game_state: Dict):
        """Render match end screen with results."""
        self.screen.fill(self.BG_DARK)
        
        # Annonce vainqueur
        if self.winner == "AI":
            winner_text = "VICTOIRE IA"
            winner_color = self.AI_GLOW
        else:
            winner_text = "VICTOIRE HUMAIN"
            winner_color = self.HUMAN_GLOW
        
        winner_surface = self.font_title.render(winner_text, True, winner_color)
        winner_rect = winner_surface.get_rect(center=(self.width // 2, self.height // 2 - 100))
        self.screen.blit(winner_surface, winner_rect)
        
        # Final scores
        ai_score = game_state.get('robot_4_hits_inflicted', 0)
        human_score = game_state.get('robot_5_hits_inflicted', 0)
        
        scores_text = f"{ai_score}  -  {human_score}"
        scores_surface = self.font_large.render(scores_text, True, self.TEXT_LIGHT)
        scores_rect = scores_surface.get_rect(center=(self.width // 2, self.height // 2))
        self.screen.blit(scores_surface, scores_rect)
        
        # Étiquettes
        ai_label = self.font_medium.render("IA", True, self.AI_PRIMARY)
        human_label = self.font_medium.render("HUMAIN", True, self.HUMAN_PRIMARY)
        
        self.screen.blit(ai_label, (self.width // 2 - 150, self.height // 2 + 60))
        self.screen.blit(human_label, (self.width // 2 + 50, self.height // 2 + 60))
        
        # Indice redémarrage
        restart_text = "Appuyez sur R pour redémarrer"
        restart_surface = self.font_small.render(restart_text, True, self.TEXT_GRAY)
        restart_rect = restart_surface.get_rect(center=(self.width // 2, self.height // 2 + 150))
        self.screen.blit(restart_surface, restart_rect)
    
    def cleanup(self):
        """Cleanup Pygame resources."""
        pygame.quit()
