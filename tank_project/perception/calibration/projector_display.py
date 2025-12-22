"""
Affichage Projecteur pour Étalonnage - VERSION FINALE

Affiche les marqueurs ArUco sur le projecteur pour l'assistant d'étalonnage.
Fonctionnalités :
- Positionnement automatique sur l'écran secondaire (VGA/HDMI)
- Mode san bordure pour masquer les décorations de fenêtre
- Curseur masqué pour l'immersion

Logs : préfixe [PROJ_DISPLAY]
"""

import os
import pygame
import cv2
import numpy as np
from typing import Tuple, Optional


class ProjectorDisplay:
    """
    Fenêtre Pygame pour projeter les motifs d'étalonnage.
    Force l'affichage sur le moniteur secondaire en utilisant les variables d'environnement SDL.
    """
    
    def __init__(self, width=1024, height=768, margin=50, monitor_offset_x=1920, monitor_offset_y=0,
                 borderless=True, hide_cursor=True):
        """
        Initialise l'affichage projecteur.
        
        Args:
            width: Largeur résolution projecteur (ex: 1024 pour VGA)
            height: Hauteur résolution projecteur (ex: 768 pour VGA)
            margin: Marge de sécurité depuis les bords (px)
            monitor_offset_x: Position X du projecteur (généralement largeur écran principal, ex: 1920)
            monitor_offset_y: Position Y (généralement 0)
            borderless: Utilise mode fenêtre sans bordure (NOFRAME)
            hide_cursor: Masque le curseur souris
        """
        self.width = width
        self.height = height
        self.margin = margin
        
        # --- CONFIGURATION MULTI-ECRAN ---
        self.monitor_x = monitor_offset_x
        self.monitor_y = monitor_offset_y
        self.borderless = borderless
        self.hide_cursor = hide_cursor
        
        self.screen = None
        self.running = False
        
        # Rectangle arène (avec marges)
        self.arena_x1 = margin
        self.arena_y1 = margin
        self.arena_x2 = width - margin
        self.arena_y2 = height - margin
        self.arena_w = self.arena_x2 - self.arena_x1
        self.arena_h = self.arena_y2 - self.arena_y1
        
        print("[PROJ_DISPLAY] Init : {}x{}, marge={}px".format(width, height, margin))
        print("[PROJ_DISPLAY] Décalage Moniteur Cible : X={}, Y={}".format(self.monitor_x, self.monitor_y))
    
    def start(self):
        """Démarre Pygame et crée la fenêtre sur le projecteur."""
        
        # 1. LE HACK: On force la position avant l'init de l'écran
        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (self.monitor_x, self.monitor_y)
        
        pygame.init()
        
        # 2. Configure les drapeaux de fenêtre selon paramètres
        flags = pygame.DOUBLEBUF
        if self.borderless:
            flags |= pygame.NOFRAME
        
        self.screen = pygame.display.set_mode((self.width, self.height), flags)
        
        # 3. IMMERSION - Masque curseur si configuré
        if self.hide_cursor:
            pygame.mouse.set_visible(False)
        
        pygame.display.set_caption("Arène Tank - Vue Projecteur")
        self.running = True
        
        mode_str = "borderless" if self.borderless else "windowed"
        print(f"[PROJ_DISPLAY] Fenêtre ouverte au décalage {self.monitor_x} ({mode_str})")
    
    def stop(self):
        """Ferme l'affichage."""
        if self.running:
            pygame.quit()
            self.running = False
            print("[PROJ_DISPLAY] Affichage fermé")
    
    def clear(self, color=(0, 0, 0)):
        """Efface l'écran avec une couleur unie."""
        if self.screen:
            self.screen.fill(color)
    
    def get_events(self):
        """
        Retourne les événements au contrôleur externe (Wizard).
        Utilisez ceci au lieu de handle_events quand la logique est contrôlée à l'extérieur.
        """
        if not self.running:
            return []
        return pygame.event.get()

    def show_corner_markers(self, marker_size_px=200, aruco_dict=cv2.aruco.DICT_4X4_100):
        """
        Affiche les marqueurs ArUco aux 4 coins de l'arène.
        """
        if not self.running:
            return
        
        # Efface avec fond blanc
        self.clear((255, 255, 255))
        
        # Génère marqueurs
        aruco_dict_obj = cv2.aruco.getPredefinedDictionary(aruco_dict)
        
        # Positions coins (centre du marqueur)
        corners = {
            0: (self.arena_x1 + marker_size_px // 2, self.arena_y2 - marker_size_px // 2),  # Bottom-left
            1: (self.arena_x2 - marker_size_px // 2, self.arena_y2 - marker_size_px // 2),  # Bottom-right
            2: (self.arena_x2 - marker_size_px // 2, self.arena_y1 + marker_size_px // 2),  # Top-right
            3: (self.arena_x1 + marker_size_px // 2, self.arena_y1 + marker_size_px // 2),  # Top-left
        }
        
        print("[PROJ_DISPLAY] Projection de 4 marqueurs de coin (IDs 0-3)")
        
        for marker_id, (cx, cy) in corners.items():
            # Génère image marqueur
            marker_img = cv2.aruco.generateImageMarker(aruco_dict_obj, marker_id, marker_size_px)
            
            # Convertit en surface pygame
            marker_img_rgb = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2RGB)
            marker_surface = pygame.surfarray.make_surface(
                np.transpose(marker_img_rgb, (1, 0, 2))
            )
            
            # Calcule coin haut-gauche
            x = cx - marker_size_px // 2
            y = cy - marker_size_px // 2
            
            # Dessine marqueur
            self.screen.blit(marker_surface, (x, y))
            
            # Ajoute étiquette ID sous marqueur
            font = pygame.font.Font(None, 36)
            text = font.render(f"ID {marker_id}", True, (0, 0, 0))
            text_rect = text.get_rect(center=(cx, cy + marker_size_px // 2 + 30))
            self.screen.blit(text, text_rect)
        
        # Met à jour affichage
        pygame.display.flip()
        
        print("[PROJ_DISPLAY] Marqueurs de coin affichés")
    
    def show_white_screen(self):
        """Affiche écran blanc uni (pour détection obstacles)."""
        if not self.running:
            return
        
        self.clear((255, 255, 255))
        
        # Ajoute instruction texte
        font = pygame.font.Font(None, 48)
        text = font.render("Placez des obstacles dans l'arène", True, (0, 0, 0))
        text_rect = text.get_rect(center=(self.width // 2, 100))
        self.screen.blit(text, text_rect)
        
        pygame.display.flip()
        print("[PROJ_DISPLAY] Écran blanc affiché")
    
    def show_message(self, message: str, color=(255, 255, 255), bg_color=(0, 0, 0)):
        """
        Affiche un message texte.
        """
        if not self.running:
            return
        
        self.clear(bg_color)
        
        # Ajustement taille police selon longueur
        font_size = 48
        if len(message) > 30:
            font_size = 36
            
        font = pygame.font.Font(None, font_size)
        text = font.render(message, True, color)
        text_rect = text.get_rect(center=(self.width // 2, self.height // 2))
        self.screen.blit(text, text_rect)
        
        pygame.display.flip()
    
    def _pump_events(self):
        """
        Interne : Garde la fenêtre réactive sans consommer les événements.
        Appelez ceci dans les opérations longues si vous n'utilisez pas get_events().
        """
        pygame.event.pump()
