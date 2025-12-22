#!/usr/bin/env python3
"""
Check Homography & Grid - Outil de Validation Visuelle (CORRIGÉ)

Cet outil permet de vérifier la correspondance entre le Monde Réel et le Monde Virtuel.
Il utilise la configuration du projecteur pour s'afficher exactement comme le jeu.

Commandes :
    [D]   : Basculer l'affichage de l'Inflation (Costmap / Grille brute)
    [ESC] : Quitter
"""

import sys
import os  # Ajouté pour SDL hack
import yaml
import pygame
import numpy as np
from pathlib import Path

# Ajout du chemin racine pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.camera.realsense_stream import RealSenseStream
from perception.camera.aruco_detector import ArucoDetector
from core.world.world_model import WorldModel
from core.world.coordinate_frames import TransformManager

# Couleurs
C_BG = (10, 10, 15)
C_GRID = (40, 40, 50)
C_OBSTACLE = (200, 50, 50)       # Rouge (Mur physique)
C_INFLATION = (200, 100, 0)      # Orange (Zone de danger)
C_MARKER = (0, 255, 255)         # Cyan (Position calculée du marqueur)
C_THEORY = (0, 255, 0)           # Vert (Coins théoriques)
C_TEXT = (255, 255, 255)

def load_config():
    """Charge les configurations nécessaires."""
    config_dir = Path(__file__).parent.parent / 'config'
    configs = {}
    for name in ['arena', 'camera', 'robot', 'projector']:
        path = config_dir / f'{name}.yaml'
        if path.exists():
            with open(path) as f:
                configs[name] = yaml.safe_load(f)
    return configs

def main():
    print("[CHECK] Démarrage validation Homographie...")

    # 1. Configs
    configs = load_config()
    if 'arena' not in configs:
        print("[CHECK] ERREUR: Config arena.yaml manquante. Lancez la calibration.")
        return

    # --- CONFIGURATION AFFICHAGE (Mise à jour pour projecteur) ---
    if 'projector' in configs:
        proj_conf = configs['projector']['projector']
        disp_conf = configs['projector']['display']
        
        win_w = proj_conf['width']
        win_h = proj_conf['height']
        margin = proj_conf['margin_px']
        
        monitor_x = disp_conf.get('monitor_offset_x', 0)
        monitor_y = disp_conf.get('monitor_offset_y', 0)
        
        print(f"[CHECK] Mode Projecteur : {win_w}x{win_h} (Offset: {monitor_x},{monitor_y})")
    else:
        # Fallback si pas de projector.yaml (mode fenêtre simple sur PC)
        print("[CHECK] Mode Fenêtré (projector.yaml introuvable)")
        win_w, win_h = 1024, 768
        margin = 50
        monitor_x, monitor_y = 0, 0

    # 2. Vision
    cam_cfg = configs['camera'].get('realsense', {})
    camera = RealSenseStream(width=848, height=480, fps=60) 
    camera.start()
    aruco = ArucoDetector()
    
    transform_mgr = TransformManager()
    if 'transform' in configs['arena'] and 'H_C2W' in configs['arena']['transform']:
        transform_mgr.H_C2W = np.array(configs['arena']['transform']['H_C2W'])
    else:
        print("[CHECK] ERREUR: Pas de matrice H_C2W dans arena.yaml")
        camera.stop()
        return

    # 3. World (pour l'inflation)
    arena_stats = configs['arena']['arena']
    robot_radius = configs['arena'].get('robot', {}).get('radius_m', 0.09)
    inflation_margin = 0.05
    
    world = WorldModel(
        arena_width_m=arena_stats['width_m'],
        arena_height_m=arena_stats['height_m'],
        grid_resolution_m=configs['arena']['grid']['resolution_m'],
        robot_radius_m=robot_radius,
        inflation_margin_m=inflation_margin
    )
    world.generate_costmap()

    # 4. Pygame Setup (Avec SDL Hack)
    # On force la position AVANT d'initier l'écran
    if monitor_x != 0 or monitor_y != 0:
        os.environ['SDL_VIDEO_WINDOW_POS'] = f"{monitor_x},{monitor_y}"
    
    pygame.init()
    
    # On utilise RESIZABLE pour la stabilité, comme dans run_game
    screen = pygame.display.set_mode((win_w, win_h), pygame.RESIZABLE)
    pygame.display.set_caption("Calibration Check - [D] Toggle Inflation")
    font = pygame.font.SysFont("Consolas", 18)

    # --- CALCUL D'ÉCHELLE (Doit être identique à PygameRenderer) ---
    draw_w = win_w - 2 * margin
    draw_h = win_h - 2 * margin
    
    scale_x = draw_w / world.arena_width
    scale_y = draw_h / world.arena_height
    scale = min(scale_x, scale_y) # Conserver aspect ratio

    def to_screen(x_m, y_m):
        """Convertit mètres -> pixels en respectant les marges du projecteur."""
        px = margin + int(x_m * scale)
        py = margin + int((world.arena_height - y_m) * scale) # Inversion Y classique
        return px, py

    # Boucle Principale
    running = True
    show_inflation = False

    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT: running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE: running = False
                    elif event.key == pygame.K_d: show_inflation = not show_inflation

            # Vision
            color_frame, _ = camera.get_frames()
            detections = aruco.detect(color_frame) if color_frame is not None else {}

            # Rendu Fond
            screen.fill(C_BG)
            
            # Grille 10cm
            for x in np.arange(0, world.arena_width, 0.1):
                p1 = to_screen(x, 0); p2 = to_screen(x, world.arena_height)
                pygame.draw.line(screen, C_GRID, p1, p2, 1)
            for y in np.arange(0, world.arena_height, 0.1):
                p1 = to_screen(0, y); p2 = to_screen(world.arena_width, y)
                pygame.draw.line(screen, C_GRID, p1, p2, 1)

            # Obstacles
            grid_data = world.grid.costmap if (show_inflation and hasattr(world.grid, 'costmap')) else world.grid.grid
            
            # Optimisation: affichage sparse
            occupied_indices = np.argwhere(grid_data > 0.5)
            cell_size_px = int(world.grid.resolution * scale) + 1
            
            for r, c in occupied_indices:
                xm, ym = world.grid.grid_to_world(r, c)
                sx, sy = to_screen(xm, ym)
                # Centrer le rectangle sur le point
                rect = pygame.Rect(sx - cell_size_px//2, sy - cell_size_px//2, cell_size_px, cell_size_px)
                pygame.draw.rect(screen, C_INFLATION if show_inflation else C_OBSTACLE, rect)

            # Coins Théoriques (Vert) - Pour vérifier l'alignement physique
            corners = [(0,0), (world.arena_width,0), (world.arena_width, world.arena_height), (0, world.arena_height)]
            for cx, cy in corners:
                pygame.draw.circle(screen, C_THEORY, to_screen(cx, cy), 10, 2)

            # ArUco Live (Cyan)
            for mid, data in detections.items():
                u, v = data['center']
                # Transformation Homographique
                xm, ym = transform_mgr.camera_to_world(u, v)
                sx, sy = to_screen(xm, ym)
                
                # Dessin cible
                pygame.draw.circle(screen, C_MARKER, (sx, sy), 8, 2)
                pygame.draw.line(screen, C_MARKER, (sx-10, sy), (sx+10, sy), 1)
                pygame.draw.line(screen, C_MARKER, (sx, sy-10), (sx, sy+10), 1)
                
                lbl = font.render(f"ID {mid}", True, C_TEXT)
                screen.blit(lbl, (sx+12, sy-10))

            # UI Text
            mode_txt = "COSTMAP (GONFLÉE)" if show_inflation else "OBSTACLES BRUTS"
            ui = font.render(f"[D] Vue: {mode_txt}", True, (0, 255, 0))
            screen.blit(ui, (10, 10))

            pygame.display.flip()

    except KeyboardInterrupt: pass
    finally:
        camera.stop()
        pygame.quit()

if __name__ == '__main__':
    main()
