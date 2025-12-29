#!/usr/bin/env python3
"""

Le robot s'oriente automatiquement vers l'ennemi.
Une fois aligné (avec tolérance), il simule un tir.

Commandes :
    [ESC] : Arrêt d'urgence et quitter
"""

import sys
import os
import yaml
import pygame
import numpy as np
import cv2
import time
import math
import json
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.camera.realsense_stream import RealSenseStream
from perception.camera.aruco_detector import ArucoDetector
from core.world.unified_transform import load_calibration
from core.control.ros_bridge_client import ROSBridgeClient

# Couleurs
C_BG = (0, 0, 0)                 # Noir
C_MARKER = (0, 255, 255)         # Cyan (Robots)
C_LASER = (0, 255, 0)            # Vert (Laser)
C_TEXT = (255, 255, 255)         # Blanc

# IDs des robots
ROBOT_AI_ID = 5      # Robot qui vise
ROBOT_ENEMY_ID = 4   # Robot cible 

def load_projector_config():
    """Charge la config projecteur pour la fenêtre."""
    config_dir = Path(__file__).parent.parent / 'config'
    path = config_dir / 'projector.yaml'
    if path.exists():
        with open(path) as f:
            return yaml.safe_load(f)
    return None

def load_robot_config():
    """Charge la config robot pour les limites de vitesse."""
    config_dir = Path(__file__).parent.parent / 'config'
    path = config_dir / 'robot.yaml'
    if path.exists():
        with open(path) as f:
            return yaml.safe_load(f)
    return None

def get_robot_pose_in_world(detection_data, transform_mgr):
    """Calcule (x, y, theta) complet dans le référentiel Monde."""
    u, v = detection_data['center']
    theta_pix = detection_data['orientation']
    
    # Point devant pour calcul angle
    dist_px = 20
    u_front = u + dist_px * np.cos(theta_pix)
    v_front = v + dist_px * np.sin(theta_pix)
    
    x, y = transform_mgr.camera_to_world(u, v)
    x_front, y_front = transform_mgr.camera_to_world(u_front, v_front)
    
    theta_world = np.arctan2(y_front - y, x_front - x)
    return x, y, theta_world

def send_shoot_command(bridge, robot_id):
    """Envoie une commande de tir via le pont ROS."""
    if not bridge or not bridge.connected:
        return
    message = {"type": "shoot", "robot_id": robot_id, "timestamp": time.time()}
    try:
        bridge.ws.send(json.dumps(message))
    except Exception as e:
        print(f"[SHOOT] Erreur envoi commande tir: {e}")


def main():
    print("[TEST] ========== Auto-Aim & Shoot (via TrajectoryFollower) ==========")
    
    # 1. Chargement Calibration
    config_dir = Path(__file__).parent.parent / 'config'
    transform_mgr = load_calibration(str(config_dir))
    
    if not transform_mgr.is_calibrated():
        print("\n[ERREUR CRITIQUE] Aucune calibration trouvée !")
        return

    # 2. Configuration Affichage & Robot
    proj_conf = load_projector_config()
    robot_conf = load_robot_config() or {}
    
    if proj_conf:
        win_w = proj_conf['projector']['width']
        win_h = proj_conf['projector']['height']
        off_x = proj_conf['display'].get('monitor_offset_x', 0)
        off_y = proj_conf['display'].get('monitor_offset_y', 0)
    else:
        win_w = transform_mgr.proj_width
        win_h = transform_mgr.proj_height
        off_x, off_y = 1920, 0

    os.environ['SDL_VIDEO_WINDOW_POS'] = f"{off_x},{off_y}"
    
    pygame.init()
    screen = pygame.display.set_mode((win_w, win_h), pygame.NOFRAME | pygame.DOUBLEBUF)
    pygame.display.set_caption("Auto Aim Shoot")
    font = pygame.font.SysFont("Consolas", 24, bold=True)
    large_font = pygame.font.SysFont("Consolas", 48, bold=True)

    # 3. Caméra
    camera = RealSenseStream(width=1280, height=720, fps=30)
    camera.start()
    time.sleep(1.0)

    K, D = camera.get_intrinsics_matrix()
    aruco = ArucoDetector()
    

    # 5. ROS Bridge
    ros_host = '127.0.0.1' 
    ros_port = 8765
    
    ros_bridge = ROSBridgeClient(host=ros_host, port=ros_port)
    robot_connected = ros_bridge.connect(max_retries=2, retry_interval=1.0)
    
    clock = pygame.time.Clock()
    running = True
    
    last_shot_time = 0
    refire_delay = 1.0
    last_log_time = time.time()
    
    # État Projectiles
    projectiles = []  # Liste de dict: {'pos': [x, y], 'vel': [vx, vy], 'color': color}
    PROJECTILE_SPEED = 500.0  # Pixels par seconde
    
    print("\n[TEST] ========================================")
    print(f"[TEST] Placez AI (ID {ROBOT_AI_ID}) et Enemy (ID {ROBOT_ENEMY_ID})")
    print("[TEST] Le TrajectoryFollower va aligner le robot.")
    print("[TEST] ========================================\n")

    print("[DEBUG] Entrée dans la boucle principale...")
    
    last_loop_time = time.time()
    
    try:
        while running:
            # DT for animation
            current_time = time.time()
            dt = current_time - last_loop_time
            last_loop_time = current_time
            
            # Events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False

            # Vision
            color_frame, _ = camera.get_frames()
            if color_frame is None:
                if time.time() - last_log_time > 2.0:
                    print("[INFO] Pas de frame caméra...")
                    last_log_time = time.time()
                continue

            if K is not None and D is not None:
                color_frame = cv2.undistort(color_frame, K, D)

            detections = aruco.detect(color_frame)
            
            ai_pose = None
            enemy_pose = None
            
            if ROBOT_AI_ID in detections:
                ai_pose = get_robot_pose_in_world(detections[ROBOT_AI_ID], transform_mgr)
            
            if ROBOT_ENEMY_ID in detections:
                enemy_pose = get_robot_pose_in_world(detections[ROBOT_ENEMY_ID], transform_mgr)

            # Rendu Background
            screen.fill(C_BG)
            
            # --- MISES A JOUR & COLLISION ---
            
            # Position visuelle des robots
            center_ai_px = None
            center_en_px = None
            hit_status = False
            
            if ai_pose:
                center_ai_px = transform_mgr.world_to_projector(ai_pose[0], ai_pose[1])
            if enemy_pose:
                center_en_px = transform_mgr.world_to_projector(enemy_pose[0], enemy_pose[1])

            # Mise à jour des projectiles
            active_projectiles = []
            ENEMY_HIT_RADIUS_PX = 100 # Rayon pour toucher (augmenté)
            
            for p in projectiles:
                p['pos'][0] += p['vel'][0] * dt
                p['pos'][1] += p['vel'][1] * dt
                
                # Check Collision avec Enemy
                hit = False
                if center_en_px:
                    dist_sq = (p['pos'][0] - center_en_px[0])**2 + (p['pos'][1] - center_en_px[1])**2
                    if dist_sq < ENEMY_HIT_RADIUS_PX**2:
                        hit = True
                        hit_status = True
                        print(f"[HIT] IMPACT CONFIRMÉ ! (Distance au centre: {math.sqrt(dist_sq):.1f}px)")
                        # Effet visuel d'impact (petit flash rouge sur l'ennemi)
                        pygame.draw.circle(screen, (255, 50, 50), (int(center_en_px[0]), int(center_en_px[1])), 45, 3)

                # Supprimer si hors écran ou touché
                px, py = p['pos']
                if not hit and (0 <= px <= win_w and 0 <= py <= win_h):
                    active_projectiles.append(p)
            projectiles = active_projectiles

            # Logique de visée
            if ai_pose and enemy_pose:
                # Calcul angle vers cible
                dx = enemy_pose[0] - ai_pose[0]
                dy = enemy_pose[1] - ai_pose[1]
                angle_to_target = math.atan2(dy, dx)
                angle_err = (angle_to_target - ai_pose[2] + math.pi) % (2 * math.pi) - math.pi
                angle_err_deg = math.degrees(angle_err)
                
                # --- VISUALISATION CANON & ROBOTS ---
                
                # Cercle autour de l'IA (Bleu Cyan)
                pygame.draw.circle(screen, C_MARKER, (int(center_ai_px[0]), int(center_ai_px[1])), 100, 2)
                
                # Cercle autour de l'Ennemi (Rouge ou Orange)
                col_enemy = (255, 100, 0) if not hit_status else (255, 0, 0)
                pygame.draw.circle(screen, col_enemy, (int(center_en_px[0]), int(center_en_px[1])), 100, 2)
                
                ai_u, ai_v = detections[ROBOT_AI_ID]['center']
                ai_px_center = transform_mgr.camera_to_projector(ai_u, ai_v)
                theta_ai = ai_pose[2]
                
                barrel_len = 50
                barrel_width = 10
                
                # Calcul des 4 coins du canon pour un rectangle orienté
                c_cos = math.cos(theta_ai)
                c_sin = math.sin(theta_ai)
                
                # Base du canon
                base_offset = -10
                bx = ai_px_center[0] + base_offset * c_cos
                by = ai_px_center[1] + base_offset * c_sin
                
                # Bout du canon
                tx = ai_px_center[0] + barrel_len * c_cos
                ty = ai_px_center[1] + barrel_len * c_sin
                
                # Vecteur perpendiculaire pour la largeur
                px = -c_sin * (barrel_width / 2)
                py = c_cos * (barrel_width / 2)
                
                poly_pts = [
                    (bx + px, by + py), # Base Gauche
                    (tx + px, ty + py), # Bout Gauche
                    (tx - px, ty - py), # Bout Droite
                    (bx - px, by - py)  # Base Droite
                ]
                
                pygame.draw.polygon(screen, (80, 80, 80), poly_pts)      # Corps gris
                pygame.draw.polygon(screen, (200, 200, 200), poly_pts, 2) # Contour clair
                
                # Pivot Central (Tourelle)
                pygame.draw.circle(screen, (40, 40, 40), (int(ai_px_center[0]), int(ai_px_center[1])), 15)
                pygame.draw.circle(screen, (100, 100, 100), (int(ai_px_center[0]), int(ai_px_center[1])), 7)
                
                # --- LOGIQUE DE TIR (CONTINU) ---
                if time.time() - last_shot_time > refire_delay:
                    if robot_connected:
                        send_shoot_command(ros_bridge, ROBOT_AI_ID)
                    last_shot_time = time.time()
                    
                    vel_x = c_cos * PROJECTILE_SPEED
                    vel_y = c_sin * PROJECTILE_SPEED
                    
                    projectiles.append({
                        'pos': [tx, ty],
                        'vel': [vel_x, vel_y],
                        'color': C_LASER
                    })
                
                # --- CONTROL MOTEUR (Simple P-Controller) ---
                # Rotation vers l'ennemi
                Kp = 2.0
                omega_cmd = Kp * angle_err
                omega_cmd = max(min(omega_cmd, 2.0), -2.0) # Saturation
                v_cmd = 0.0 # On reste sur place
                
                status_txt = f"Reviewing Targets... err={angle_err_deg:.1f}°"
                color_status = C_LASER
                
                # Ligne de visée (visuelle)
                pygame.draw.line(screen, (255, 50, 50), ai_px_center, center_en_px, 1)

                if robot_connected:
                    ros_bridge.send_velocity_command(ROBOT_AI_ID, v_cmd, omega_cmd)
                    
            else:
                if robot_connected:
                    ros_bridge.send_velocity_command(ROBOT_AI_ID, 0.0, 0.0)
                status_txt = "WAITING FOR ROBOTS..."
                color_status = C_TEXT
                
                if time.time() - last_log_time > 2.0:
                    print(f"[INFO] En attente de détection... (AI={ROBOT_AI_ID in detections}, Enemy={ROBOT_ENEMY_ID in detections})")
                    last_log_time = time.time()

            # --- Rendu Projectiles ---
            for p in projectiles:
                pygame.draw.circle(screen, p['color'], (int(p['pos'][0]), int(p['pos'][1])), 8)
                pygame.draw.circle(screen, (255, 255, 255), (int(p['pos'][0]-2), int(p['pos'][1]-2)), 3)

            # --- Robot Markers ---

            # --- Visualisation (Reste inchangé) ---
            # Robots
            for mid, data in detections.items():
                if mid in [ROBOT_AI_ID, ROBOT_ENEMY_ID]:
                    u, v = data['center']
                    px, py = transform_mgr.camera_to_projector(u, v)
                    pygame.draw.circle(screen, C_MARKER, (int(px), int(py)), 15, 2)
                    
                    lbl = "AI" if mid == ROBOT_AI_ID else "TARGET"
                    screen.blit(font.render(lbl, True, C_TEXT), (px+20, py-20))

            # UI Text
            txt_surf = large_font.render(status_txt, True, color_status)
            screen.blit(txt_surf, (50, 50))
            
            pygame.display.flip()
            clock.tick(30)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        import traceback
        traceback.print_exc()
        print(f"[ERREUR] Exception non gérée : {e}")
    finally:
        print("[TEST] Fin.")
        if robot_connected:
            ros_bridge.send_velocity_command(ROBOT_AI_ID, 0.0, 0.0)
            ros_bridge.disconnect()
        camera.stop()
        pygame.quit()

if __name__ == '__main__':
    main()

