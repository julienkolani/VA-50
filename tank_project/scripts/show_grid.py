#!/usr/bin/env python3
"""
Check Homography & Grid - Outil de Validation Visuelle

Cet outil permet de vérifier la correspondance entre le Monde Réel et le Monde Virtuel.
A lancer APRES la calibration et AVANT le jeu.

Commandes :
    [D]   : Basculer l'affichage de l'Inflation (Costmap / Grille brute)
    [ESC] : Quitter
"""

import sys
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

def main():
    print("[CHECK] Démarrage validation Homographie...")

    # 1. Configs
    config_dir = Path(__file__).parent.parent / 'config'
    try:
        with open(config_dir / 'arena.yaml') as f: arena_cfg = yaml.safe_load(f)
        with open(config_dir / 'camera.yaml') as f: camera_cfg = yaml.safe_load(f)
        with open(config_dir / 'robot.yaml') as f: robot_cfg = yaml.safe_load(f)
    except FileNotFoundError:
        print("[CHECK] ERREUR: Config manquante. Lancez la calibration d'abord.")
        return

    # 2. Vision
    cam_cfg = camera_cfg.get('realsense', {})
    camera = RealSenseStream(width=848, height=480, fps=60) 
    camera.start()
    aruco = ArucoDetector()
    
    transform_mgr = TransformManager()
    if 'transform' in arena_cfg and 'H_C2W' in arena_cfg['transform']:
        transform_mgr.H_C2W = np.array(arena_cfg['transform']['H_C2W'])
    else:
        print("[CHECK] ERREUR: Pas de matrice H_C2W dans arena.yaml")
        camera.stop()
        return

    # 3. World (pour l'inflation)
    # On récupère le rayon du robot depuis robot.yaml (plus fiable que arena.yaml qui est écrasé)
    robot_radius = robot_cfg.get('physical', {}).get('robot_radius_m', 0.09)
    inflation_margin = 0.05 # Marge par défaut
    
    world = WorldModel(
        arena_width_m=arena_cfg['arena']['width_m'],
        arena_height_m=arena_cfg['arena']['height_m'],
        grid_resolution_m=arena_cfg['grid']['resolution_m'],
        robot_radius_m=robot_radius,
        inflation_margin_m=inflation_margin
    )
    world.generate_costmap()

    # 4. Pygame
    pygame.init()
    info = pygame.display.Info()
    win_h = int(info.current_h * 0.8)
    aspect_ratio = world.arena_width / world.arena_height
    win_w = int(win_h * aspect_ratio)
    screen = pygame.display.set_mode((win_w, win_h))
    pygame.display.set_caption("Calibration Check - [D] Toggle Inflation")
    font = pygame.font.SysFont("Consolas", 18)

    scale_px = win_w / world.arena_width

    def to_screen(x_m, y_m):
        sx = int(x_m * scale_px)
        sy = int(win_h - (y_m * scale_px)) # Flip Y
        return sx, sy

    # Boucle
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
            rows, cols = grid_data.shape
            cell_w_px = world.grid.resolution * scale_px
            
            # Optimisation: affichage sparse
            occupied_indices = np.argwhere(grid_data > 0.5)
            for r, c in occupied_indices:
                xm, ym = world.grid.grid_to_world(r, c)
                sx, sy = to_screen(xm, ym)
                rect = pygame.Rect(sx - cell_w_px/2, sy - cell_w_px/2, cell_w_px+1, cell_w_px+1)
                pygame.draw.rect(screen, C_INFLATION if show_inflation else C_OBSTACLE, rect)

            # Coins Théoriques (Vert)
            corners = [(0,0), (world.arena_width,0), (world.arena_width, world.arena_height), (0, world.arena_height)]
            for cx, cy in corners:
                pygame.draw.circle(screen, C_THEORY, to_screen(cx, cy), 10, 2)

            # ArUco Live (Cyan)
            for mid, data in detections.items():
                u, v = data['center']
                xm, ym = transform_mgr.camera_to_world(u, v)
                sx, sy = to_screen(xm, ym)
                
                pygame.draw.circle(screen, C_MARKER, (sx, sy), 8, 0)
                pygame.draw.line(screen, (0,0,0), (sx-8, sy), (sx+8, sy), 1)
                pygame.draw.line(screen, (0,0,0), (sx, sy-8), (sx, sy+8), 1)
                
                lbl = font.render(f"ID {mid}", True, C_TEXT)
                screen.blit(lbl, (sx+12, sy-10))

            # UI
            mode_txt = "COSTMAP" if show_inflation else "RAW"
            ui = font.render(f"[D] View: {mode_txt}", True, (0, 255, 0))
            screen.blit(ui, (10, 10))

            pygame.display.flip()

    except KeyboardInterrupt: pass
    finally:
        camera.stop()
        pygame.quit()

if __name__ == '__main__':
    main()
