
"""
Moteur de Rendu Pygame - Visualisation de Jeu Moderne (Compatible AR)
"""
import pygame
import numpy as np
import os
import cv2
from typing import Dict, Tuple, Optional, List

class PygameRenderer:
    # États du match
    STATE_WAITING = "waiting"
    STATE_COUNTDOWN = "countdown"
    STATE_PLAYING = "playing"
    STATE_FINISHED = "finished"
    
    def __init__(self, width: int = 1024, height: int = 768, margin: int = 50, 
                 monitor_offset_x: int = 0, monitor_offset_y: int = 0, borderless: bool = True,
                 transform_mgr=None):
        """
        Initialise le rendu.
        Args:
            transform_mgr: Instance UnifiedTransform pour conversion précise Monde -> Ecran
        """
        print(f"[VIS] Configuration écran : {width}x{height} à ({monitor_offset_x},{monitor_offset_y})")
        os.environ['SDL_VIDEO_WINDOW_POS'] = f"{monitor_offset_x},{monitor_offset_y}"
        
        pygame.init()
        
        self.width = width
        self.height = height
        self.margin = margin
        self.transform_mgr = transform_mgr  # Utilisation de la calibration exacte
        
        # Création fenêtre
        flags = pygame.DOUBLEBUF
        if borderless: flags |= pygame.NOFRAME
        
        try:
            self.screen = pygame.display.set_mode((self.width, self.height), flags)
        except pygame.error:
            self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)

        pygame.display.set_caption("Tank Arena - Projector View")
        pygame.font.init()
        # Fallback fonts
        try:
             self.font_large = pygame.font.SysFont('Arial Black', 48, bold=True)
             self.font_medium = pygame.font.SysFont('Arial', 36, bold=True)
        except:
             self.font_large = pygame.font.SysFont('Arial', 48, bold=True)
             self.font_medium = pygame.font.SysFont('Arial', 36, bold=True)
        
        # Couleurs
        self.AI_PRIMARY = (50, 150, 255)
        self.HUMAN_PRIMARY = (255, 80, 80)
        self.ACCENT_GOLD = (255, 200, 50)
        self.ACCENT_GREEN = (50, 255, 150)
        self.TEXT_DARK = (30, 30, 35)
        
        # État
        self.match_state = self.STATE_WAITING
        self.show_debug_path = False
        self.ai_path = []
        self.scale = 100.0 # Fallback scale
        self.shot_effects = [] # Liste des effets de tir actifs

    def set_arena_dimensions(self, width_m: float, height_m: float):
        # Fallback pour le calcul d'échelle si pas de transform_mgr
        draw_w = self.width - 2 * self.margin
        draw_h = self.height - 2 * self.margin
        self.scale = min(draw_w / width_m, draw_h / height_m)

    def world_to_screen(self, x_m: float, y_m: float) -> Tuple[int, int]:
        """Convertit monde (mètres) -> écran (pixels) avec précision."""
        if self.transform_mgr and self.transform_mgr.is_calibrated():
            # Utilise la matrice exacte de calibration (comme show_grid)
            return self.transform_mgr.world_to_projector(x_m, y_m)
        else:
            # Fallback approximatif (moins précis)
            px = self.margin + int(x_m * self.scale)
            # Attention à l'inversion Y ici si pas de transform manager
            # On suppose que 0,0 est en bas à gauche dans le monde
            # et en haut à gauche dans Pygame, d'où l'inversion nécessaire
            # Mais transform_mgr le gère déjà mieux.
            return (px, int((self.height/self.scale - y_m) * self.scale) + self.margin) # Simplifié approximate

    def handle_keypress(self, event: pygame.event.Event):
        if event.type == pygame.KEYDOWN and event.key == pygame.K_d:
            self.show_debug_path = not self.show_debug_path

    def start_match(self): self.match_state = self.STATE_PLAYING # Simplifié pour debug
    def end_match(self, winner): self.match_state = self.STATE_FINISHED

    def set_ai_path(self, waypoints): self.ai_path = waypoints

    def add_shot_fx(self, start_pose, distance, color=None):
        """Ajoute un effet visuel de tir."""
        x, y, theta = start_pose
        # Calcul du point de départ (un peu décalé pour ne pas sortir du centre du robot)
        start_x = x + 0.2 * np.cos(theta)
        start_y = y + 0.2 * np.sin(theta)
        
        end_x = start_x + distance * np.cos(theta)
        end_y = start_y + distance * np.sin(theta)
        
        self.shot_effects.append({
            'start': (start_x, start_y),
            'end': (end_x, end_y),
            'life': 10,  # Durée de vie en frames (approx 0.16s à 60fps)
            'color': color
        })

    def render_frame(self, world_state: Dict, game_state: Dict, background_image=None):
        self.screen.fill((0, 0, 0)) # Fond NOIR par défaut pour projecteur

        # 1. DESSINER LE FOND CAMÉRA (AR)
        if background_image is not None:
            # Conversion OpenCV (BGR) -> Pygame (RGB)
            frame_rgb = cv2.cvtColor(background_image, cv2.COLOR_BGR2RGB)
            frame_rgb = np.transpose(frame_rgb, (1, 0, 2))
            surf = pygame.surfarray.make_surface(frame_rgb)
            
            # Afficher (l'image est déjà warpée par run_game.py)
            self.screen.blit(surf, (0, 0))
            
            # Assombrir légèrement pour lisibilité
            dark = pygame.Surface((self.width, self.height))
            dark.set_alpha(80)
            dark.fill((0,0,0))
            self.screen.blit(dark, (0,0))

        # 2. Obstacles (Debug visuel)
        if 'occupancy_grid' in world_state and (self.show_debug_path or background_image is None):
            grid_obj = world_state['occupancy_grid']
            if hasattr(grid_obj, 'grid'):
                grid_surf = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
                
                rows, cols = grid_obj.grid.shape
                cell_px = int(self.scale * grid_obj.resolution)
                
                obstacles = np.argwhere(grid_obj.grid > 50) # Seuil obstacle
                
                for r, c in obstacles:
                    xm, ym = grid_obj.grid_to_world(r, c)
                    px, py = self.world_to_screen(xm, ym)
                    pygame.draw.rect(grid_surf, (255, 0, 0, 100), (px, py, cell_px, cell_px))
                
                self.screen.blit(grid_surf, (0,0))

        # 3. Robots
        r4 = world_state.get('robot_4_pose', (0,0,0))
        r5 = world_state.get('robot_5_pose', (0,0,0))
        self._draw_robot(r4, self.AI_PRIMARY, "IA")
        self._draw_robot(r5, self.HUMAN_PRIMARY, "P1")

        # 4. Debug Path
        if self.show_debug_path and self.ai_path:
            pts = [self.world_to_screen(x, y) for x, y in self.ai_path]
            if len(pts) > 1:
                pygame.draw.lines(self.screen, self.ACCENT_GREEN, False, pts, 3)

        # 5. Effets de Tirs (VFX)
        for i in range(len(self.shot_effects) - 1, -1, -1):
            fx = self.shot_effects[i]
            if fx['life'] > 0:
                p1 = self.world_to_screen(*fx['start'])
                p2 = self.world_to_screen(*fx['end'])
                
                # Couleur qui fade out avec la vie
                if fx.get('color'):
                     # Fade out alpha using provided color
                     c = fx['color']
                     # Simulated alpha fade by darkening? Or just keep simple
                     # Pygame line doesn't support alpha easily without surface.
                     # Just keep distinct color.
                     color = c
                else:
                    # Default yellow->red
                    color = (255, 255 if fx['life'] > 5 else 0, 0)
                
                pygame.draw.line(self.screen, color, p1, p2, 4)
                fx['life'] -= 1
            else:
                self.shot_effects.pop(i)

        # 5. UI
        self._draw_hud(game_state)
        pygame.display.flip()

    def _draw_robot(self, pose, color, label):
        x, y, th = pose
        px, py = self.world_to_screen(x, y)
        pygame.draw.circle(self.screen, color, (px, py), 20)
        # Ligne de direction
        
        # Astuce : Pour savoir la direction visuelle, on projette un point "devant" le robot
        if self.transform_mgr:
             # Point 20cm devant le robot
             fx, fy = x + 0.2*np.cos(th), y + 0.2*np.sin(th)
             fpx, fpy = self.world_to_screen(fx, fy)
             pygame.draw.line(self.screen, (255,255,255), (px, py), (fpx, fpy), 3)
        else:
             end_x = px + 30 * np.cos(th)
             end_y = py - 30 * np.sin(th) # Y inversé simple
             pygame.draw.line(self.screen, (255,255,255), (px, py), (end_x, end_y), 3)

    def _draw_hud(self, gs):
        # Score simple (toujours visible)
        s1 = self.font_medium.render(f"IA: {gs.get('robot_4_hits_inflicted', 0)}", True, self.AI_PRIMARY)
        self.screen.blit(s1, (20, 20))
        
        # --- HUD DEBUG ÉTENDU ---
        if self.show_debug_path:
            y_offset = 80
            line_height = 25
            
            # Fond semi-transparent pour le texte
            bg_rect = pygame.Rect(10, 70, 350, 250)
            s = pygame.Surface((bg_rect.width, bg_rect.height), pygame.SRCALPHA)
            s.fill((0, 0, 0, 180))
            self.screen.blit(s, (10, 70))
            
            # Helper pour dessiner du texte
            def draw_text(text, color=(200, 200, 200)):
                nonlocal y_offset
                surf = pygame.font.SysFont('Arial', 18, bold=True).render(text, True, color)
                self.screen.blit(surf, (20, y_offset))
                y_offset += line_height

            draw_text("--- DEBUG MODE (Toggle 'D') ---", self.ACCENT_GOLD)
            
            # Infos Match
            draw_text(f"État Jeu: {self.match_state}")
            draw_text(f"Temps: {gs.get('time_remaining_s', 0):.1f}s")
            
            # Infos Robot IA
            if 'robot_4_pose' in gs: # Note: on passe world_state et game_state souvent mélangés
                 # TODO: run_game.py doit passer ces infos dans le dict
                 pass
            
            # Comme render_frame reçoit world_state séparément, on ne l'a pas ici facilement
            # sauf si on stocke world_state dans self lors du render_frame
             

    def cleanup(self):
        pygame.quit()
