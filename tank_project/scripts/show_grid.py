#!/usr/bin/env python3
"""
Outil de Validation Grille & Homographie (CORRIGÉ)

Utilise STRICTEMENT la calibration unifiée (UnifiedTransform) pour l'affichage.
Garantit que l'affichage correspond exactement à ce que l'IA "voit".

Commandes :
    [D]   : Basculer l'affichage de l'Inflation (Costmap / Grille brute)
    [ESC] : Quitter
"""

import sys
import os
import yaml
import pygame
import numpy as np
import cv2
from pathlib import Path

# Ajout du chemin racine pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.camera.realsense_stream import RealSenseStream
from perception.camera.aruco_detector import ArucoDetector
from core.world.world_model import WorldModel
from core.world.unified_transform import UnifiedTransform, load_calibration

# Couleurs
C_BG = (0, 0, 0)                 # Noir pur pour le projecteur
C_GRID = (50, 50, 50)            # Gris foncé
C_OBSTACLE = (200, 50, 50)       # Rouge (Mur physique)
C_INFLATION = (200, 100, 0)      # Orange (Zone de danger)
C_MARKER = (0, 255, 255)         # Cyan (Position détectée)
C_TEXT = (255, 255, 255)

def load_projector_config():
    """Charge la config projecteur pour la fenêtre."""
    config_dir = Path(__file__).parent.parent / 'config'
    path = config_dir / 'projector.yaml'
    if path.exists():
        with open(path) as f:
            return yaml.safe_load(f)
    return None

def main():
    print("[CHECK] Démarrage validation Homographie...")

    # 1. Chargement Calibration (CRITIQUE)
    config_dir = Path(__file__).parent.parent / 'config'
    transform_mgr = load_calibration(str(config_dir))
    
    if not transform_mgr.is_calibrated():
        print("\n[ERREUR CRITIQUE] Aucune calibration trouvée !")
        print("Veuillez lancer : python -m perception.calibration.standalone_wizard")
        return

    print(f"[CHECK] Calibration chargée : {transform_mgr.pixels_per_meter:.2f} px/m")

    # 2. Configuration Affichage (Depuis calibration ou projector.yaml)
    proj_conf = load_projector_config()
    
    if proj_conf:
        win_w = proj_conf['projector']['width']
        win_h = proj_conf['projector']['height']
        off_x = proj_conf['display'].get('monitor_offset_x', 0)
        off_y = proj_conf['display'].get('monitor_offset_y', 0)
    else:
        # Fallback sur les données de calibration
        win_w = transform_mgr.proj_width
        win_h = transform_mgr.proj_height
        off_x, off_y = 1920, 0 # Valeur par défaut courante

    # Force la position de la fenêtre (SDL Hack)
    os.environ['SDL_VIDEO_WINDOW_POS'] = f"{off_x},{off_y}"
    
    pygame.init()
    screen = pygame.display.set_mode((win_w, win_h), pygame.NOFRAME | pygame.DOUBLEBUF)
    pygame.display.set_caption("Calibration Check")
    font = pygame.font.SysFont("Consolas", 24, bold=True)

    # 3. Vision
    print("[CHECK] Démarrage caméra...")
    # On récupère les dimensions depuis la config si possible, sinon défaut
    camera = RealSenseStream(width=1280, height=720, fps=30) 
    camera.start()
    import time; time.sleep(1.0) # Warmup

    # Paramètres intrinsèques pour Undistort
    K, D = camera.get_intrinsics_matrix()
    aruco = ArucoDetector()

    # 4. World Model (Pour l'inflation)
    # Chargement config robot depuis arena.yaml
    robot_radius = 0.09 # Default Turtlebot
    inflation_margin = 0.05
    
    arena_conf_path = Path(__file__).parent.parent / 'config' / 'arena.yaml'
    if arena_conf_path.exists():
        with open(arena_conf_path) as f:
            arena_conf = yaml.safe_load(f)
            if 'robot' in arena_conf:
                robot_radius = arena_conf['robot'].get('radius_m', 0.09)
                inflation_margin = arena_conf['robot'].get('inflation_margin_m', 0.05)
                print(f"[CONFIG] Robot chargé: Rayon={robot_radius}m (diam ~{robot_radius*2*100:.0f}cm), Marge={inflation_margin}m")

    # On utilise les dimensions exactes calculées par la calibration
    world = WorldModel(
        arena_width_m=transform_mgr.arena_width_m,
        arena_height_m=transform_mgr.arena_height_m,
        grid_resolution_m=0.02, # 2cm
        robot_radius_m=robot_radius,
        inflation_margin_m=inflation_margin
    )
    world.generate_costmap()

    # CHARGEMENT GRILLE OBSTACLES (Occupancy Grid)
    occupancy_path = Path(__file__).parent.parent / 'config' / 'occupancy.npy'
    if occupancy_path.exists():
        print(f"[CHECK] Chargement grille : {occupancy_path}")
        try:
            occupancy_img = np.load(occupancy_path)
            pixels_per_meter = transform_mgr.pixels_per_meter if transform_mgr.is_calibrated() else 1.0
            
            if pixels_per_meter > 0:
                occ_y, occ_x = np.where(occupancy_img > 0)
                
                # Conversion Vectorisée (Projector Pixels -> Meters)
                x_m = occ_x / pixels_per_meter
                y_m = occ_y / pixels_per_meter
                
                # Filtrage pour rester dans l'arène
                # valid_mask = (x_m >= 0) & (x_m <= transform_mgr.arena_width_m) & ... (Simplified)
                
                res = world.grid.resolution
                grid_cols = (x_m / res).astype(int)
                grid_rows = (y_m / res).astype(int)
                
                obstacle_cells = list(set(zip(grid_rows, grid_cols)))
                world.grid.set_static_obstacles(obstacle_cells)
                # Regenerate costmap after adding obstacles
                world.generate_costmap() 
                print(f"[CHECK] {len(obstacle_cells)} cellules d'obstacles injectées.")
        except Exception as e:
            print(f"[CHECK] Erreur chargement grille: {e}")

    # Boucle Principale
    # Boucle Principale
    running = True
    show_inflation = True # RE-ACTIVÉ PAR DÉFAUT (Utilisateur ne voit rien)
    clock = pygame.time.Clock()

    try:
        while running:
            # Events
            for event in pygame.event.get():
                if event.type == pygame.QUIT: running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE: running = False
                    elif event.key == pygame.K_d: 
                        show_inflation = not show_inflation

            # 1. Acquisition Image
            color_frame, _ = camera.get_frames()
            if color_frame is None: continue

            # 2. Correction Distorsion (CRITIQUE pour l'alignement sur les bords)
            if K is not None and D is not None:
                color_frame = cv2.undistort(color_frame, K, D)

            # 3. Détection
            detections = aruco.detect(color_frame)

            # 4. Rendu
            screen.fill(C_BG)

            # --- A. DESSIN DE LA GRILLE (Via UnifiedTransform) ---
            # On dessine une grille tous les 10cm pour vérifier l'échelle
            step_m = 0.10 
            
            # Lignes Verticales
            for x in np.arange(0, world.arena_width, step_m):
                # On utilise world_to_projector qui utilise la matrice exacte
                start_px = transform_mgr.world_to_projector(x, 0)
                end_px = transform_mgr.world_to_projector(x, world.arena_height)
                pygame.draw.line(screen, C_GRID, start_px, end_px, 1)

            # Lignes Horizontales
            for y in np.arange(0, world.arena_height, step_m):
                start_px = transform_mgr.world_to_projector(0, y)
                end_px = transform_mgr.world_to_projector(world.arena_width, y)
                pygame.draw.line(screen, C_GRID, start_px, end_px, 1)

            # --- B. DESSIN DES OBSTACLES (Inflation) ---
            # On n'affiche que si demandé (touche D) pour garder la grille claire
            if show_inflation:
                grid_data = world.grid.costmap
                # Optimisation : trouver les indices occupés
                occ_y, occ_x = np.where(grid_data > 0.5)
                
                # Taille d'une cellule en pixels (environ)
                cell_px = int(world.grid.resolution * transform_mgr.pixels_per_meter)
                # Minimum 1px
                cell_px = max(1, cell_px)
                
                for r, c in zip(occ_y, occ_x):
                    # Conversion Grille -> Monde -> Pixels Projecteur
                    xm, ym = world.grid.grid_to_world(r, c)
                    px, py = transform_mgr.world_to_projector(xm, ym)
                    
                    rect = pygame.Rect(px - cell_px//2, py - cell_px//2, cell_px, cell_px)
                    pygame.draw.rect(screen, C_INFLATION, rect)

            # --- C. DESSIN DES ARUCO EN TEMPS RÉEL (Le Test Ultime) ---
            for mid, data in detections.items():
                u, v = data['center'] # Pixels Caméra
                
                # Transformation : Caméra -> Projecteur (Directe via Homographie)
                proj_x, proj_y = transform_mgr.camera_to_projector(u, v)
                
                # Dessin Cible
                # Si la calibration est bonne, ce cercle cyan doit être PILE SUR le robot réel
                center = (int(proj_x), int(proj_y))
                pygame.draw.circle(screen, C_MARKER, center, 10, 2)
                pygame.draw.line(screen, C_MARKER, (center[0]-15, center[1]), (center[0]+15, center[1]), 2)
                pygame.draw.line(screen, C_MARKER, (center[0], center[1]-15), (center[0], center[1]+15), 2)
                
                # Label ID
                lbl = font.render(f"ID:{mid}", True, C_TEXT)
                screen.blit(lbl, (center[0]+15, center[1]-15))

            # UI Info
            mode = "COSTMAP" if show_inflation else "GRILLE"
            info = f"[ESC] Quitter | [D] Mode: {mode} | Scale: {transform_mgr.pixels_per_meter:.1f} px/m"
            screen.blit(font.render(info, True, (0, 255, 0)), (20, 20))
            
            pygame.display.flip()
            clock.tick(60)

    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()
        pygame.quit()

if __name__ == '__main__':
    main()
