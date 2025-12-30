#!/usr/bin/env python3
"""
Test Composant : Go To Point

Test du système de navigation complet :
- Détection ArUco (Robot AI ID=4, Enemy ID=5)
- Planification de chemin A* vers l'ennemi avec distance de sécurité
- Suivi de trajectoire avec TrajectoryFollower
- Contrôle robot via ROS Bridge
- Visualisation pygame : caméra + grille + robots + path

Mode: Boucle ouverte - calcule le chemin, l'affiche, envoie les commandes, attend.

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

# Couleurs
C_BG = (0, 0, 0)                 # Noir
C_GRID = (50, 50, 50)            # Gris foncé
C_MARKER = (0, 255, 255)         # Cyan (Robots)
C_PATH = (0, 200, 100)           # Vert (Chemin)
C_WAYPOINT = (255, 255, 0)       # Jaune (Waypoint actuel)
C_TEXT = (255, 255, 255)         # Blanc

# IDs des robots
ROBOT_AI_ID = 5      # Robot qui se déplace (IA)
ROBOT_ENEMY_ID = 4   # Robot cible (fixe - adversaire)

# Distance de sécurité (en mètres)
SAFETY_DISTANCE = 0.20  # 20cm - distance de tir optimale (réduite)

# Seuil de replanification (cercle de tolérance)
REPLAN_THRESHOLD_M = 0.08  # 8cm - ne replanifie que si l'ennemi bouge > 8cm


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
    """
    Calcule (x, y, theta) complet dans le référentiel Monde.
    
    Args:
        detection_data: Dict avec 'center' et 'orientation' (pixels)
        transform_mgr: UnifiedTransform instance
        
    Returns:
        (x, y, theta) en mètres et radians
    """
    # 1. Centre du robot (Pixels)
    u, v = detection_data['center']
    
    # 2. Point "Devant" le robot (Pixels)
    # On utilise + sin car transform_mgr gère déjà la conversion d'axes
    theta_pix = detection_data['orientation']
    dist_px = 20  # 20 pixels devant
    u_front = u + dist_px * np.cos(theta_pix)
    v_front = v + dist_px * np.sin(theta_pix)  # + car transform_mgr gère la conversion
    
    # 3. Projection dans le monde (Mètres)
    x, y = transform_mgr.camera_to_world(u, v)
    x_front, y_front = transform_mgr.camera_to_world(u_front, v_front)
    
    # 4. Calcul du vrai angle Monde
    theta_world = np.arctan2(y_front - y, x_front - x)
    
    return x, y, theta_world


def main():
    print("[TEST] ========== Go To Point Test ==========")
    
    # 1. Chargement Calibration
    config_dir = Path(__file__).parent.parent / 'config'
    transform_mgr = load_calibration(str(config_dir))
    
    if not transform_mgr.is_calibrated():
        print("\n[ERREUR CRITIQUE] Aucune calibration trouvée !")
        print("Veuillez lancer : python -m perception.calibration.standalone_wizard")
        return

    print(f"[TEST] Calibration chargée : {transform_mgr.pixels_per_meter:.2f} px/m")
    
    # 2. Configuration Affichage
    proj_conf = load_projector_config()
    robot_conf = load_robot_config()
    
    if proj_conf:
        win_w = proj_conf['projector']['width']
        win_h = proj_conf['projector']['height']
        off_x = proj_conf['display'].get('monitor_offset_x', 0)
        off_y = proj_conf['display'].get('monitor_offset_y', 0)
    else:
        win_w = transform_mgr.proj_width
        win_h = transform_mgr.proj_height
        off_x, off_y = 1920, 0

    # Force la position de la fenêtre
    os.environ['SDL_VIDEO_WINDOW_POS'] = f"{off_x},{off_y}"
    
    pygame.init()
    screen = pygame.display.set_mode((win_w, win_h), pygame.NOFRAME | pygame.DOUBLEBUF)
    pygame.display.set_caption("Go To Point Test")
    font = pygame.font.SysFont("Consolas", 24, bold=True)

    # 3. Caméra et Vision
    print("[TEST] Démarrage caméra...")
    camera = RealSenseStream(width=1280, height=720, fps=30)
    camera.start()
    time.sleep(1.0)  # Warmup

    K, D = camera.get_intrinsics_matrix()
    aruco = ArucoDetector()

    # 4. World Model
    world = WorldModel(
        arena_width_m=transform_mgr.arena_width_m,
        arena_height_m=transform_mgr.arena_height_m,
        grid_resolution_m=0.02,
        robot_radius_m=0.09,
        inflation_margin_m=0.05
    )
    world.generate_costmap()
    
    # 5. Planificateur A*
    planner = AStarPlanner(world.grid, heuristic='euclidean')
    
    # 6. Contrôleur de Trajectoire
    if robot_conf:
        control_conf = robot_conf.get('control', {}).copy()
        control_conf.update(robot_conf.get('velocity_limits', {}))
    else:
        control_conf = {
            'lookahead_distance_m': 0.15,    # Réduit pour laisser Pure Pursuit guider plus longtemps
            'k_velocity': 0.15,              # Vitesse linéaire
            'k_theta': 1.2,                  # Non utilisé (remplacé par state machine)
            'waypoint_threshold_m': 0.05,    # Seuil de waypoint atteint
            'max_linear_mps': 0.20,
            'max_angular_radps': 1.0         # Réduit à 1.0 pour stabilité ArUco
        }
    
    controller = TrajectoryFollower(control_conf)
    
    # 7. ROS Bridge
    if robot_conf:
        ros_host = robot_conf['ros_bridge']['host']
        ros_port = robot_conf['ros_bridge']['port']
    else:
        ros_host = '127.0.0.1'
        ros_port = 8765
    
    ros_bridge = ROSBridgeClient(host=ros_host, port=ros_port)
    robot_connected = ros_bridge.connect(max_retries=2, retry_interval=1.0)
    
    if not robot_connected:
        print("[TEST] ATTENTION: Robot non connecté, visualisation seule")
    
    # Variables d'état
    clock = pygame.time.Clock()
    running = True
    
    # État du test
    state = "DETECTING"  # DETECTING -> PLANNING -> NAVIGATING -> REACHED
    ai_pose = None
    enemy_pose = None
    path = None
    current_waypoint_idx = 0
    tick_counter = 0
    last_log_time = time.time()
    last_enemy_pos_for_path = None  # Pour détecter si ennemi a bougé
    
    print("\n[TEST] ========================================")
    print("[TEST] Positionnez les robots:")
    print(f"  - Robot AI (ID {ROBOT_AI_ID})")
    print(f"  - Robot Enemy (ID {ROBOT_ENEMY_ID})")
    print("[TEST] Appuyez sur ESC pour arrêter")
    print("[TEST] ========================================\n")
    
    try:
        while running:
            tick_counter += 1
            current_time = time.time()
            
            # Events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        print("\n[TEST] ========================================")
                        print("[TEST] ARRÊT D'URGENCE DEMANDÉ !")
                        if robot_connected:
                            print("[TEST] Envoi commande stop au robot...")
                            ros_bridge.send_velocity_command(ROBOT_AI_ID, 0.0, 0.0)
                            print("[TEST] Robot arrêté.")
                        print("[TEST] ========================================\n")
                        running = False

            # 1. Acquisition Image
            color_frame, _ = camera.get_frames()
            if color_frame is None:
                continue

            # 2. Correction Distorsion
            if K is not None and D is not None:
                color_frame = cv2.undistort(color_frame, K, D)

            # 3. Détection
            detections = aruco.detect(color_frame)

            # 4. Mise à jour des poses
            ai_detected = ROBOT_AI_ID in detections
            enemy_detected = ROBOT_ENEMY_ID in detections
            
            if ai_detected:
                x, y, theta = get_robot_pose_in_world(detections[ROBOT_AI_ID], transform_mgr)
                ai_pose = (x, y, theta)
            
            if enemy_detected:
                x, y, theta = get_robot_pose_in_world(detections[ROBOT_ENEMY_ID], transform_mgr)
                enemy_pose = (x, y, theta)
            
            # Log détection toutes les 2 secondes en mode DETECTING
            if state == "DETECTING" and (current_time - last_log_time) > 2.0:
                if ai_detected and enemy_detected:
                    print(f"[DETECT] [OK] AI et Enemy détectés")
                elif ai_detected:
                    print(f"[DETECT] [OK] AI détecté | [NON] Enemy manquant")
                elif enemy_detected:
                    print(f"[DETECT] [NON] AI manquant | [OK] Enemy détecté")
                else:
                    print(f"[DETECT] [NON] Aucun robot détecté")
                last_log_time = current_time

            # 5. Machine à états du test
            if state == "DETECTING":
                if ai_pose is not None and enemy_pose is not None:
                    print("\n[TEST] ========================================")
                    print("[TEST] [OK] ROBOTS DÉTECTÉS !")
                    print(f"[TEST]   AI Pose    : ({ai_pose[0]:.3f}, {ai_pose[1]:.3f}, θ={np.degrees(ai_pose[2]):.1f}°)")
                    print(f"[TEST]   Enemy Pose : ({enemy_pose[0]:.3f}, {enemy_pose[1]:.3f}, θ={np.degrees(enemy_pose[2]):.1f}°)")
                    
                    dist_initial = np.sqrt((ai_pose[0] - enemy_pose[0])**2 + (ai_pose[1] - enemy_pose[1])**2)
                    print(f"[TEST]   Distance   : {dist_initial:.3f}m")
                    print("[TEST] ========================================\n")
                    state = "PLANNING"
            
            elif state == "PLANNING":
                if ai_pose and enemy_pose:
                    # Mémoriser où était l'ennemi au moment du calcul
                    last_enemy_pos_for_path = np.array([enemy_pose[0], enemy_pose[1]])
                    
                    print("[PLAN] Calcul de la cible avec distance de sécurité...")
                    
                    # Calculer le point cible avec distance de sécurité
                    dx = ai_pose[0] - enemy_pose[0]
                    dy = ai_pose[1] - enemy_pose[1]
                    dist = np.sqrt(dx**2 + dy**2)
                    
                    # Direction de l'enemy vers l'AI
                    if dist > 0.01:
                        ux = dx / dist
                        uy = dy / dist
                    else:
                        ux, uy = 1.0, 0.0
                    
                    # Point à distance de sécurité de l'enemy
                    target_x = enemy_pose[0] + ux * SAFETY_DISTANCE
                    target_y = enemy_pose[1] + uy * SAFETY_DISTANCE
                    target_pos = (target_x, target_y)
                    
                    print(f"[PLAN] Position cible : ({target_x:.3f}, {target_y:.3f})")
                    print(f"[PLAN] Distance à parcourir : {dist - SAFETY_DISTANCE:.3f}m")
                    print(f"[PLAN] Lancement A* pathfinding...")
                    
                    # Planifier le chemin
                    start_time = time.time()
                    full_path = planner.plan(ai_pose[:2], target_pos)
                    planning_time = time.time() - start_time
                    
                    if full_path and len(full_path) > 0:
                        # Downsampling : garder 1 point sur 4 pour fluidité (tous les ~8-10cm)
                        path = full_path[::4] if len(full_path) > 4 else full_path
                        # S'assurer que la cible finale est bien dans le path
                        if path[-1] != target_pos:
                            path.append(target_pos)
                        
                        print(f"[PLAN] [OK] Chemin trouvé en {planning_time*1000:.1f}ms")
                        print(f"[PLAN] Nombre de waypoints : {len(full_path)} → {len(path)} (downsampled)")
                        print(f"[PLAN] Premier waypoint : ({path[0][0]:.3f}, {path[0][1]:.3f})")
                        print(f"[PLAN] Dernier waypoint : ({path[-1][0]:.3f}, {path[-1][1]:.3f})")
                        current_waypoint_idx = 0
                        print("\n[NAV] Début de la navigation...\n")
                        state = "NAVIGATING"
                    else:
                        print("[PLAN] [NON] ATTENTION: Impossible de planifier le chemin !")
                        print("[PLAN] Utilisation waypoint direct (fallback)...")
                        path = [target_pos]
                        current_waypoint_idx = 0
                        print("\n[NAV] Début de la navigation (mode direct)...\n")
                        state = "NAVIGATING"
            
            elif state == "NAVIGATING":
                if ai_pose and path and enemy_pose:
                    # ======== BOUCLE FERMÉE : Vérifier si l'ennemi a bougé ========
                    if last_enemy_pos_for_path is not None:
                        curr_enemy_pos = np.array([enemy_pose[0], enemy_pose[1]])
                        enemy_movement = np.linalg.norm(curr_enemy_pos - last_enemy_pos_for_path)
                        
                        if enemy_movement > REPLAN_THRESHOLD_M:
                            print(f"\n[LOOP] [ATTENTION] L'adversaire a bougé de {enemy_movement*100:.1f}cm > {REPLAN_THRESHOLD_M*100:.0f}cm")
                            print("[LOOP] → Replanification du chemin...")
                            state = "PLANNING"
                            continue
                    
                    # Vérifier si destination atteinte (tous waypoints passés)
                    if current_waypoint_idx >= len(path):
                        print("\n[NAV] ========================================")
                        print("[NAV] [OK] TOUS LES WAYPOINTS ATTEINTS !")
                        if robot_connected:
                            print("[NAV] Arrêt du robot...")
                            ros_bridge.send_velocity_command(ROBOT_AI_ID, 0.0, 0.0)
                        
                        # Calcul distance finale
                        if enemy_pose:
                            final_dist = np.sqrt((ai_pose[0] - enemy_pose[0])**2 + (ai_pose[1] - enemy_pose[1])**2)
                            print(f"[NAV] Distance finale à l'ennemi : {final_dist:.3f}m")
                            print(f"[NAV] Écart par rapport à la cible : {abs(final_dist - SAFETY_DISTANCE)*100:.1f}cm")
                        print("[NAV] ========================================\n")
                        state = "REACHED"
                    else:
                        # Calculer distance au waypoint actuel
                        wp = path[current_waypoint_idx]
                        dist_to_wp = np.sqrt((ai_pose[0] - wp[0])**2 + (ai_pose[1] - wp[1])**2)
                        
                        # Si waypoint atteint, avancer
                        if dist_to_wp < 0.05:  # 5cm
                            print(f"[NAV] [OK] Waypoint {current_waypoint_idx + 1}/{len(path)} atteint ({wp[0]:.3f}, {wp[1]:.3f})")
                            current_waypoint_idx += 1
                            if current_waypoint_idx >= len(path):
                                print("\n[NAV] ========================================")
                                print("[NAV] [OK] TOUS LES WAYPOINTS ATTEINTS !")
                                if robot_connected:
                                    print("[NAV] Arrêt du robot...")
                                    ros_bridge.send_velocity_command(ROBOT_AI_ID, 0.0, 0.0)
                                
                                # Calcul distance finale
                                if enemy_pose:
                                    final_dist = np.sqrt((ai_pose[0] - enemy_pose[0])**2 + (ai_pose[1] - enemy_pose[1])**2)
                                    print(f"[NAV] Distance finale à l'ennemi : {final_dist:.3f}m")
                                    print(f"[NAV] Écart par rapport à la cible : {abs(final_dist - SAFETY_DISTANCE)*100:.1f}cm")
                                print("[NAV] ========================================\n")
                                state = "REACHED"
                        else:
                            # Calculer commandes de contrôle (CRITIQUE: passer le chemin futur)
                            future_path = path[current_waypoint_idx:]
                            
                            # ===== LOGS DÉTAILLÉS =====
                            # Position actuelle
                            print(f"\n[NAV] ========== Navigation Step ==========")
                            print(f"[NAV] Pose actuelle : x={ai_pose[0]:.3f}m, y={ai_pose[1]:.3f}m, θ={np.degrees(ai_pose[2]):.1f}°")
                            
                            # Waypoint actuel
                            print(f"[NAV] Waypoint actuel [{current_waypoint_idx+1}/{len(path)}] : x={wp[0]:.3f}m, y={wp[1]:.3f}m")
                            print(f"[NAV] Distance au waypoint : {dist_to_wp:.3f}m")
                            
                            # Afficher les 3 prochains waypoints
                            print(f"[NAV] Chemin restant ({len(future_path)} points) :")
                            for i, wp_preview in enumerate(future_path[:min(3, len(future_path))]):
                                print(f"[NAV]   WP[{current_waypoint_idx+i+1}]: ({wp_preview[0]:.3f}, {wp_preview[1]:.3f})")
                            if len(future_path) > 3:
                                print(f"[NAV]   ... et {len(future_path)-3} autres points")
                            
                            # Distance au goal final
                            final_goal = path[-1]
                            dist_to_goal = np.sqrt((ai_pose[0] - final_goal[0])**2 + (ai_pose[1] - final_goal[1])**2)
                            print(f"[NAV] Distance au GOAL final : {dist_to_goal:.3f}m")
                            
                            # Commandes de contrôle
                            v_cmd, omega_cmd = controller.compute_control(ai_pose, future_path)
                            print(f"[NAV] Commande : v={v_cmd:.3f} m/s, ω={omega_cmd:.3f} rad/s")
                            print(f"[NAV] ==========================================\n")
                            
                            # CONDITION D'ARRÊT ABSOLU : Si très proche de la cible finale
                            if enemy_pose:
                                dist_to_enemy = np.sqrt((ai_pose[0] - enemy_pose[0])**2 + (ai_pose[1] - enemy_pose[1])**2)
                                if dist_to_enemy < 0.05:  # Moins de 5cm de l'ennemi
                                    v_cmd = 0.0
                                    omega_cmd = 0.0
                                    print(f"[NAV] [OK] Distance cible atteinte ({dist_to_enemy*100:.1f}cm), arrêt absolu")
                            
                            # Log périodique des commandes (toutes les 1 secondes)
                            if tick_counter % 30 == 0:
                                print(f"[NAV] WP {current_waypoint_idx + 1}/{len(path)} | Pose: ({ai_pose[0]:.3f}, {ai_pose[1]:.3f}) | Dist: {dist_to_wp:.3f}m | v={v_cmd:.3f} ω={omega_cmd:.3f}")
                            
                            # Envoyer au robot
                            if robot_connected:
                                ros_bridge.send_velocity_command(ROBOT_AI_ID, v_cmd, omega_cmd)
            
            elif state == "REACHED":
                # Arrêt confirmé
                if robot_connected:
                    ros_bridge.send_velocity_command(ROBOT_AI_ID, 0.0, 0.0)
                
                # ======== BOUCLE FERMÉE : Surveiller si l'ennemi s'éloigne ========
                if enemy_pose and last_enemy_pos_for_path is not None:
                    curr_enemy_pos = np.array([enemy_pose[0], enemy_pose[1]])
                    enemy_movement = np.linalg.norm(curr_enemy_pos - last_enemy_pos_for_path)
                    
                    if enemy_movement > REPLAN_THRESHOLD_M:
                        print(f"\n[LOOP] [ATTENTION] L'adversaire s'est éloigné de {enemy_movement*100:.1f}cm")
                        print("[LOOP] → Nouvelle planification...")
                        state = "PLANNING"

            # 6. Rendu Pygame
            screen.fill(C_BG)

            # Dessin de la grille du maillage A* (lignes rouges)
            # Utilise la résolution du WorldModel pour afficher les cellules exactes
            grid_step_m = world.grid.resolution  # 0.02m (2cm) - résolution du A*
            grid_color = (255, 0, 0)  # Rouge pour bien visualiser
            
            # Lignes verticales
            for x in np.arange(0, world.arena_width, grid_step_m):
                start_px = transform_mgr.world_to_projector(x, 0)
                end_px = transform_mgr.world_to_projector(x, world.arena_height)
                pygame.draw.line(screen, grid_color, start_px, end_px, 1)

            # Lignes horizontales
            for y in np.arange(0, world.arena_height, grid_step_m):
                start_px = transform_mgr.world_to_projector(0, y)
                end_px = transform_mgr.world_to_projector(world.arena_width, y)
                pygame.draw.line(screen, grid_color, start_px, end_px, 1)

            # Dessin du chemin
            if path and len(path) > 1:
                for i in range(len(path) - 1):
                    p1 = transform_mgr.world_to_projector(path[i][0], path[i][1])
                    p2 = transform_mgr.world_to_projector(path[i+1][0], path[i+1][1])
                    pygame.draw.line(screen, C_PATH, p1, p2, 3)
            
            # Dessin du waypoint actuel
            if path and state == "NAVIGATING" and current_waypoint_idx < len(path):
                wp = path[current_waypoint_idx]
                wp_px = transform_mgr.world_to_projector(wp[0], wp[1])
                pygame.draw.circle(screen, C_WAYPOINT, (int(wp_px[0]), int(wp_px[1])), 8, 3)

            # Dessin des robots détectés
            for mid, data in detections.items():
                if mid in [ROBOT_AI_ID, ROBOT_ENEMY_ID]:
                    u, v = data['center']
                    proj_x, proj_y = transform_mgr.camera_to_projector(u, v)
                    
                    center = (int(proj_x), int(proj_y))
                    pygame.draw.circle(screen, C_MARKER, center, 10, 2)
                    pygame.draw.line(screen, C_MARKER, (center[0]-15, center[1]), (center[0]+15, center[1]), 2)
                    pygame.draw.line(screen, C_MARKER, (center[0], center[1]-15), (center[0], center[1]+15), 2)
                    
                    label_text = "AI" if mid == ROBOT_AI_ID else "ENEMY"
                    lbl = font.render(f"{label_text}", True, C_TEXT)
                    screen.blit(lbl, (center[0]+15, center[1]-15))

            # UI Info
            status_text = f"État: {state}"
            if ai_pose and enemy_pose:
                dist = np.sqrt((ai_pose[0] - enemy_pose[0])**2 + (ai_pose[1] - enemy_pose[1])**2)
                status_text += f" | Distance: {dist:.2f}m"
            
            info_lines = [
                "[ESC] Arrêt",
                status_text,
                f"Safety Distance: {SAFETY_DISTANCE}m"
            ]
            
            y_offset = 20
            for line in info_lines:
                txt = font.render(line, True, (0, 255, 0))
                screen.blit(txt, (20, y_offset))
                y_offset += 30

            pygame.display.flip()
            clock.tick(30)  # 30 FPS

    except KeyboardInterrupt:
        print("[TEST] Interruption clavier")
    finally:
        print("\n[TEST] Nettoyage...")
        if robot_connected:
            ros_bridge.send_velocity_command(ROBOT_AI_ID, 0.0, 0.0)
            ros_bridge.disconnect()
        camera.stop()
        pygame.quit()
        
        # WAIT à la fin comme demandé
        print("[TEST] Test terminé. Appuyez sur Entrée pour fermer...")
        input()


if __name__ == '__main__':
    main()
