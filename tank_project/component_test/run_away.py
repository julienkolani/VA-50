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
from core.world.world_model import WorldModel
from core.world.unified_transform import UnifiedTransform, load_calibration
from core.ia.planners.a_star import AStarPlanner
from core.control.trajectory_follower import TrajectoryFollower
from core.control.ros_bridge_client import ROSBridgeClient

# --- CONFIGURATION DESIGN ---
C_BG = (5, 5, 15)
C_TACTICAL_GRID = (30, 30, 60)
C_SAFE_ZONE = (0, 255, 100)        # Vert (Refuge)
C_PATH = (0, 255, 150)             # Vert (Trajectoire)
C_DANGER = (255, 50, 50)           # Rouge (Ennemi)
C_CYAN = (0, 255, 255)             # IA
C_GOLD = (255, 215, 0)             # Animation
C_WHITE = (255, 255, 255)

# --- PARAMÈTRES TACTIQUES ---
ROBOT_AI_ID = 5
ROBOT_ENEMY_ID = 4
PANIC_DISTANCE = 0.60     # Rayon d'alerte (60cm)
REPLAN_THRESHOLD_M = 0.10  # Re-calculer si l'ennemi bouge de 10cm
SEGMENTS_X, SEGMENTS_Y = 5, 4 # Pour le choix de cible stratégique

class FloatingText:
    def __init__(self, pos_px, text):
        self.x, self.y = pos_px
        self.text = text
        self.lifetime = 45 
        self.alpha = 255
        self.font = pygame.font.SysFont("Impact", 52)

    def update(self):
        self.y -= 2
        self.alpha = max(0, self.alpha - 6)
        self.lifetime -= 1

    def draw(self, surface):
        s = self.font.render(self.text, True, C_GOLD)
        s.set_alpha(self.alpha)
        surface.blit(s, (self.x - s.get_width()//2, self.y))

def get_robot_pose_in_world(detection_data, transform_mgr):
    u, v = detection_data['center']
    theta_pix = detection_data['orientation']
    u_front = u + 20 * np.cos(theta_pix)
    v_front = v + 20 * np.sin(theta_pix)
    x, y = transform_mgr.camera_to_world(u, v)
    x_f, y_f = transform_mgr.camera_to_world(u_front, v_front)
    # Protection contre les bords de l'arène (évite les erreurs d'index négatif)
    x = np.clip(x, 0.01, transform_mgr.arena_width_m - 0.01)
    y = np.clip(y, 0.01, transform_mgr.arena_height_m - 0.01)
    return x, y, np.arctan2(y_f - y, x_f - x)

def main():
    print("[SYSTEM] Démarrage du module Tactical Flee (All Fixes Applied)...")
    
    # 1. Initialisation
    config_dir = Path(__file__).parent.parent / 'config'
    transform_mgr = load_calibration(str(config_dir))
    proj_conf = {}
    if (config_dir / 'projector.yaml').exists():
        with open(config_dir / 'projector.yaml') as f: proj_conf = yaml.safe_load(f)

    # Hack SDL pour projecteur
    off_x = proj_conf.get('display', {}).get('monitor_offset_x', 1920)
    os.environ['SDL_VIDEO_WINDOW_POS'] = f"{off_x},0"

    pygame.init()
    screen = pygame.display.set_mode((1024, 768), pygame.NOFRAME)
    font_hud = pygame.font.SysFont("Consolas", 20, bold=True)

    camera = RealSenseStream(width=1280, height=720, fps=30)
    camera.start()
    aruco = ArucoDetector()
    K, D = camera.get_intrinsics_matrix()

    # WorldModel avec grille fine de 2cm
    world = WorldModel(transform_mgr.arena_width_m, transform_mgr.arena_height_m)
    world.generate_costmap() 
    planner = AStarPlanner(world.grid)
    
    controller = TrajectoryFollower({
        'lookahead_distance_m': 0.15,
        'k_velocity': 0.7,
        'waypoint_threshold_m': 0.08,
        'max_linear_mps': 0.22,
        'max_angular_radps': 1.2
    })
    
    ros_bridge = ROSBridgeClient(host='127.0.0.1', port=8765)
    robot_connected = ros_bridge.connect()

    state = "SCANNING" 
    path = []
    current_waypoint_idx = 0
    flee_target = None
    last_enemy_pos_for_path = None
    floating_texts = []
    arena_center_px = transform_mgr.world_to_projector(world.arena_width/2, world.arena_height/2)

    try:
        while True:
            screen.fill(C_BG)
            current_time = time.time()
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE: return

            color_frame, _ = camera.get_frames()
            if color_frame is None: continue
            if K is not None: color_frame = cv2.undistort(color_frame, K, D)
            detections = aruco.detect(color_frame)

            ai_pose = enemy_pose = None
            if ROBOT_AI_ID in detections: 
                ai_pose = get_robot_pose_in_world(detections[ROBOT_AI_ID], transform_mgr)
            if ROBOT_ENEMY_ID in detections: 
                enemy_pose = get_robot_pose_in_world(detections[ROBOT_ENEMY_ID], transform_mgr)

            # --- LOGIQUE TACTIQUE ---
            if ai_pose and enemy_pose:
                dist_enemy = np.hypot(ai_pose[0]-enemy_pose[0], ai_pose[1]-enemy_pose[1])

                if state == "FLEEING" and last_enemy_pos_for_path:
                    moved = np.hypot(enemy_pose[0]-last_enemy_pos_for_path[0], enemy_pose[1]-last_enemy_pos_for_path[1])
                    if moved > REPLAN_THRESHOLD_M: state = "SCANNING"

                if state == "SCANNING":
                    if dist_enemy < PANIC_DISTANCE:
                        # FIX: Utilisation du bon nom d'attribut 'robot_radius_m'
                        world.grid.update_dynamic_obstacles([enemy_pose], world.robot_radius_m)
                        last_enemy_pos_for_path = (enemy_pose[0], enemy_pose[1])
                        
                        max_d, best_zone = -1, None
                        sx, sy = world.arena_width / SEGMENTS_X, world.arena_height / SEGMENTS_Y
                        for ix in range(SEGMENTS_X):
                            for iy in range(SEGMENTS_Y):
                                zx, zy = (ix + 0.5) * sx, (iy + 0.5) * sy
                                # FIX: Utilisation de is_position_valid du WorldModel
                                if world.is_position_valid(zx, zy): 
                                    d = np.hypot(zx - enemy_pose[0], zy - enemy_pose[1])
                                    if d > max_d: max_d, best_zone = d, (zx, zy)
                        
                        if best_zone:
                            flee_target = best_zone
                            full_path = planner.plan(ai_pose[:2], flee_target)
                            if full_path:
                                path = full_path[::2] 
                                if path[-1] != flee_target: path.append(flee_target)
                                current_waypoint_idx = 0
                                state = "FLEEING"

                elif state == "FLEEING":
                    if current_waypoint_idx >= len(path):
                        floating_texts.append(FloatingText(arena_center_px, "+1 ESCAPE"))
                        state = "SCANNING"
                        if robot_connected: ros_bridge.send_velocity_command(ROBOT_AI_ID, 0, 0)
                        continue

                    wp = path[current_waypoint_idx]
                    if controller.is_waypoint_reached(ai_pose, wp):
                        current_waypoint_idx += 1
                        if current_waypoint_idx >= len(path):
                            floating_texts.append(FloatingText(arena_center_px, "+1 ESCAPE"))
                            state = "SCANNING"
                            if robot_connected: ros_bridge.send_velocity_command(ROBOT_AI_ID, 0, 0)
                            continue
                    
                    v, w = controller.compute_control(ai_pose, path[current_waypoint_idx:])
                    if robot_connected: ros_bridge.send_velocity_command(ROBOT_AI_ID, v, w)

            # --- RENDU VISUEL ---
            sx, sy = world.arena_width / SEGMENTS_X, world.arena_height / SEGMENTS_Y
            for ix in range(SEGMENTS_X):
                for iy in range(SEGMENTS_Y):
                    p1 = transform_mgr.world_to_projector(ix*sx, iy*sy)
                    p2 = transform_mgr.world_to_projector((ix+1)*sx, (iy+1)*sy)
                    pygame.draw.rect(screen, C_TACTICAL_GRID, (p1[0], p1[1], p2[0]-p1[0], p2[1]-p1[1]), 1)

            if path and state == "FLEEING":
                path_px = [transform_mgr.world_to_projector(p[0], p[1]) for p in path[current_waypoint_idx:]]
                if len(path_px) > 1:
                    pygame.draw.lines(screen, C_PATH, False, path_px, 3)

            if flee_target and state == "FLEEING":
                target_px = transform_mgr.world_to_projector(flee_target[0], flee_target[1])
                pygame.draw.circle(screen, C_SAFE_ZONE, target_px, 40, 2)

            if ai_pose: pygame.draw.circle(screen, C_CYAN, transform_mgr.world_to_projector(ai_pose[0], ai_pose[1]), 15, 2)
            if enemy_pose: pygame.draw.circle(screen, C_DANGER, transform_mgr.world_to_projector(enemy_pose[0], enemy_pose[1]), 15, 2)

            for ft in floating_texts[:]:
                ft.update()
                ft.draw(screen)
                if ft.lifetime <= 0: floating_texts.remove(ft)

            pygame.display.flip()
            pygame.time.Clock().tick(30)

    except KeyboardInterrupt: pass
    finally:
        if robot_connected: ros_bridge.send_velocity_command(ROBOT_AI_ID, 0, 0)
        camera.stop()
        pygame.quit()

if __name__ == '__main__': main()