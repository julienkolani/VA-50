#!/usr/bin/env python3
import sys
import os
import yaml
import pygame
import numpy as np
import cv2
import time
from pathlib import Path

# Ajout du chemin racine pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.camera.realsense_stream import RealSenseStream
from perception.camera.aruco_detector import ArucoDetector
from core.world.unified_transform import UnifiedTransform, load_calibration
from core.control.ros_bridge_client import ROSBridgeClient

# Configuration Couleurs & Design
C_BG = (5, 5, 15)
C_LASER = (255, 0, 0)
C_LOCK = (0, 255, 0)
C_HUD = (0, 255, 255)
C_GOLD = (255, 215, 0)

# Paramètres de Tir
ROBOT_AI_ID = 5
ROBOT_ENEMY_ID = 4
ALIGN_PRECISION_DEG = 3.0  # Seuil de tir
SHOOT_COOLDOWN = 1.5       # Temps entre deux tirs
ROTATION_GAIN = 1.5        # Force de la rotation sur place

class FloatingScore:
    """Animation +1 PT au centre de l'arène"""
    def __init__(self, pos_px):
        self.x, self.y = pos_px
        self.lifetime = 45 # frames
        self.alpha = 255
        self.font = pygame.font.SysFont("Impact", 42)

    def update(self):
        self.y -= 1.5
        self.alpha = max(0, self.alpha - 6)
        self.lifetime -= 1

    def draw(self, surface):
        s = self.font.render("+1 POINT", True, C_GOLD)
        s.set_alpha(self.alpha)
        surface.blit(s, (self.x - s.get_width()//2, self.y))

def get_robot_pose_in_world(detection_data, transform_mgr):
    u, v = detection_data['center']
    theta_pix = detection_data['orientation']
    dist_px = 20
    u_front = u + dist_px * np.cos(theta_pix)
    v_front = v + dist_px * np.sin(theta_pix)
    x, y = transform_mgr.camera_to_world(u, v)
    x_f, y_f = transform_mgr.camera_to_world(u_front, v_front)
    return x, y, np.arctan2(y_f - y, x_f - x)

def main():
    print("[SYSTEM] Démarrage du module Sniper...")
    
    # 1. Chargement Calibration & Config
    config_dir = Path(__file__).parent.parent / 'config'
    transform_mgr = load_calibration(str(config_dir))
    proj_conf = {}
    path_proj = config_dir / 'projector.yaml'
    if path_proj.exists():
        with open(path_proj) as f: proj_conf = yaml.safe_load(f)

    # 2. Hack SDL Position Fenêtre
    off_x = proj_conf.get('display', {}).get('monitor_offset_x', 1920)
    off_y = proj_conf.get('display', {}).get('monitor_offset_y', 0)
    os.environ['SDL_VIDEO_WINDOW_POS'] = f"{off_x},{off_y}"

    # 3. Initialisation Pygame
    pygame.init()
    win_w = proj_conf.get('projector', {}).get('width', 1024)
    win_h = proj_conf.get('projector', {}).get('height', 768)
    screen = pygame.display.set_mode((win_w, win_h), pygame.NOFRAME)
    font_hud = pygame.font.SysFont("Consolas", 24, bold=True)

    # 4. Vision & Hardware
    camera = RealSenseStream(width=1280, height=720, fps=30)
    camera.start()
    aruco = ArucoDetector()
    K, D = camera.get_intrinsics_matrix()
    
    ros_bridge = ROSBridgeClient(host='127.0.0.1', port=8765)
    robot_connected = ros_bridge.connect()

    # Variables de jeu
    score = 0
    last_shot_time = 0
    muzzle_flash_timer = 0
    floating_scores = []
    
    # Calcul du centre de l'arène en pixels (via homographie)
    center_world_x = transform_mgr.arena_width_m / 2
    center_world_y = transform_mgr.arena_height_m / 2
    arena_center_px = transform_mgr.world_to_projector(center_world_x, center_world_y)

    try:
        while True:
            current_time = time.time()
            screen.fill(C_BG)
            
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE: return

            # Perception
            color_frame, _ = camera.get_frames()
            if color_frame is None: continue
            if K is not None: color_frame = cv2.undistort(color_frame, K, D)
            detections = aruco.detect(color_frame)

            ai_pose = enemy_pose = None
            if ROBOT_AI_ID in detections:
                ai_pose = get_robot_pose_in_world(detections[ROBOT_AI_ID], transform_mgr)
            if ROBOT_ENEMY_ID in detections:
                enemy_pose = get_robot_pose_in_world(detections[ROBOT_ENEMY_ID], transform_mgr)

            # --- LOGIQUE SNIPER ---
            is_locked = False
            error_deg = 0.0
            if ai_pose and enemy_pose:
                dx, dy = enemy_pose[0] - ai_pose[0], enemy_pose[1] - ai_pose[1]
                angle_to_enemy = np.arctan2(dy, dx)
                alpha = (angle_to_enemy - ai_pose[2] + np.pi) % (2 * np.pi) - np.pi
                error_deg = abs(np.degrees(alpha))

                # Commande de rotation pur
                w_cmd = np.clip(-(ROTATION_GAIN * alpha), -1.2, 1.2)

                if error_deg < ALIGN_PRECISION_DEG:
                    is_locked = True
                    w_cmd = 0 # Stabilisation
                    
                    if (current_time - last_shot_time) > SHOOT_COOLDOWN:
                        # --- FEU ! ---
                        score += 1
                        last_shot_time = current_time
                        muzzle_flash_timer = 6 # Durée du flash
                        floating_scores.append(FloatingScore(arena_center_px))
                        
                        if robot_connected:
                            # Utilisation de send_velocity_command pour simuler ou commande spécifique
                            print(f"[FIRE] Score: {score}")
                            try: ros_bridge.send_shoot_command(ROBOT_AI_ID)
                            except: pass

                if robot_connected:
                    ros_bridge.send_velocity_command(ROBOT_AI_ID, 0.0, w_cmd)

            # --- RENDU HUD & ANIMATIONS ---
            # 1. Grille de fond
            for x in np.arange(0, transform_mgr.arena_width_m, 0.1):
                p1 = transform_mgr.world_to_projector(x, 0)
                p2 = transform_mgr.world_to_projector(x, transform_mgr.arena_height_m)
                pygame.draw.line(screen, (20, 20, 40), p1, p2, 1)

            if ai_pose and enemy_pose:
                ai_px = transform_mgr.world_to_projector(ai_pose[0], ai_pose[1])
                en_px = transform_mgr.world_to_projector(enemy_pose[0], enemy_pose[1])
                color = C_LOCK if is_locked else C_LASER
                
                # Laser
                pygame.draw.line(screen, color, ai_px, en_px, 1 if not is_locked else 2)
                
                # Réticule Ennemi
                s = 40 + int(np.sin(time.time()*12)*5)
                pygame.draw.rect(screen, color, (en_px[0]-s//2, en_px[1]-s//2, s, s), 2)

                # Muzzle Flash (Animation de tir sur le robot AI)
                if muzzle_flash_timer > 0:
                    pygame.draw.circle(screen, (255, 255, 255), ai_px, 35)
                    pygame.draw.circle(screen, (255, 255, 0), ai_px, 20)
                    muzzle_flash_timer -= 1

            # 2. Floating Scores (Au centre de l'arène)
            for fs in floating_scores[:]:
                fs.update()
                fs.draw(screen)
                if fs.lifetime <= 0: floating_scores.remove(fs)

            # 3. HUD Infos
            hud_y = 30
            infos = [
                f"SCORE: {score:03d}",
                f"ERROR: {error_deg:.1f} DEG",
                f"STATUS: {'LOCKED' if is_locked else 'AIMING'}"
            ]
            for line in infos:
                img = font_hud.render(line, True, C_HUD)
                screen.blit(img, (30, hud_y))
                hud_y += 35

            pygame.display.flip()
            pygame.time.Clock().tick(30)

    except KeyboardInterrupt: pass
    finally:
        if robot_connected: ros_bridge.send_velocity_command(ROBOT_AI_ID, 0.0, 0.0)
        camera.stop()
        pygame.quit()

if __name__ == '__main__': main()