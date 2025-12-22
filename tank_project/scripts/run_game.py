#!/usr/bin/env python3
"""
Main Game Script - Tank Arena

Integrates all components:
- Perception (Camera, ArUco, Kalman)
- Core (Game Engine, World Model, IA)
- Visualization (Pygame Projector)
- Control (ROS Bridge)

Usage:
    python3 run_game.py
"""

import sys
import time
import yaml
import pygame
import numpy as np
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.camera.realsense_stream import RealSenseStream
from perception.camera.aruco_detector import ArucoDetector
from perception.camera.kalman_filter import KalmanFilter
from core.world.world_model import WorldModel
from core.world.coordinate_frames import TransformManager
from core.game.game_engine import GameEngine
from core.ia.strategy import AIStrategy
from core.control.trajectory_follower import TrajectoryFollower
from core.control.ros_bridge_client import ROSBridgeClient
from visualization.pygame_renderer import PygameRenderer


def load_config():
    """Charge tous les fichiers de configuration."""
    config_dir = Path(__file__).parent.parent / 'config'
    
    configs = {}
    for config_file in ['arena', 'camera', 'game', 'ia', 'robot', 'projector']:
        if (config_dir / '{}.yaml'.format(config_file)).exists():
            with open(config_dir / '{}.yaml'.format(config_file)) as f:
                configs[config_file] = yaml.safe_load(f)
    
    return configs


def setup_transform_manager(configs):
    """
    Configure la transformation de coordonnées à partir des données de calibration.
    """
    transform_mgr = TransformManager()
    
    if 'transform' in configs['arena'] and 'H_C2W' in configs['arena']['transform']:
        transform_mgr.H_C2W = np.array(configs['arena']['transform']['H_C2W'])
        print("[MAIN] Matrice d'homographie chargée avec succès.")
    else:
        print("[MAIN] ATTENTION : Pas de calibration trouvée. Utilisez 'run_calibration.py' d'abord.")
        # Fallback
        if 'scale' in configs['arena'].get('transform', {}):
             transform_mgr.set_av_to_world_scale(configs['arena']['transform']['scale'])
    
    return transform_mgr


def camera_to_world(detection_center, transform_mgr, fallback_scale=1.0):
    """
    Transform ArUco detection from camera pixels to world meters.
    
    Args:
        detection_center: (u, v) pixel coordinates
        transform_mgr: TransformManager instance
        fallback_scale: Scale to use if no homography
        
    Returns:
        (x, y) in meters
    """
    u, v = detection_center
    
    if transform_mgr.H_C2W is not None:
        return transform_mgr.camera_to_world(u, v)
    else:
        # Fallback: simple scaling from center
        x = u * fallback_scale / 1000.0
        y = v * fallback_scale / 1000.0
        return (x, y)


def get_robot_pose_in_world(detection_data, transform_mgr, fallback_scale=1.0):
    """
    Calcule (x, y, theta) complet dans le référentiel Monde.
    
    CRITIQUE: L'orientation pixel doit être transformée via homographie
    sinon le robot croira avancer vers l'Est alors qu'il va vers le Nord.
    
    Args:
        detection_data: Dict avec 'center' et 'orientation' (pixels)
        transform_mgr: TransformManager instance
        fallback_scale: Fallback si pas d'homographie
        
    Returns:
        (x, y, theta) en mètres et radians (référentiel monde)
    """
    # 1. Centre du robot (Pixels)
    u, v = detection_data['center']
    
    # 2. Point "Devant" le robot (Pixels) - utilise l'orientation pixel
    theta_pix = detection_data['orientation']
    dist_px = 20  # 20 pixels devant
    u_front = u + dist_px * np.cos(theta_pix)
    v_front = v + dist_px * np.sin(theta_pix)
    
    # 3. Projection dans le monde (Mètres)
    x, y = camera_to_world((u, v), transform_mgr, fallback_scale)
    x_front, y_front = camera_to_world((u_front, v_front), transform_mgr, fallback_scale)
    
    # 4. Calcul du vrai angle Monde
    theta_world = np.arctan2(y_front - y, x_front - x)
    
    return x, y, theta_world


def main():
    print("[MAIN] ========== Tank Arena Game ==========")
    
    # 1. Configuration
    configs = load_config()
    
    # Init subsystems
    transform_mgr = setup_transform_manager(configs)
    
    fallback_scale = 1.0
    if 'transform' in configs['arena'] and 'scale' in configs['arena']['transform']:
        fallback_scale = configs['arena']['transform']['scale']

    # 1. Perception
    camera = RealSenseStream(
        width=configs['camera']['realsense'].get('width', 848),
        height=configs['camera']['realsense'].get('height', 480),
        fps=configs['camera']['realsense'].get('fps', 60)
    )
    camera.start()
    
    aruco = ArucoDetector()
    kalman_ai = KalmanFilter()
    kalman_human = KalmanFilter()
    
    # 2. World - avec config robot
    robot_config = configs['arena'].get('robot', {})
    world = WorldModel(
        arena_width_m=configs['arena']['arena']['width_m'],
        arena_height_m=configs['arena']['arena']['height_m'],
        grid_resolution_m=configs['arena']['grid']['resolution_m'],
        robot_radius_m=robot_config.get('radius_m', 0.09),
        inflation_margin_m=robot_config.get('inflation_margin_m', 0.05)
    )
    
    # CRITIQUE: Générer la costmap AVANT de donner la grille à l'IA
    # Sinon le robot frôlera les murs
    world.generate_costmap()
    
    # 3. Game
    game_engine = GameEngine(configs['game'])
    
    # 4. IA (utilise maintenant la costmap gonflée)
    ai_strategy = AIStrategy(configs['ia'])
    ai_strategy.set_planner(world.grid)
    
    # 5. Control (ROS Bridge)
    controller = TrajectoryFollower(configs['robot'].get('control', {}))
    ros_bridge = ROSBridgeClient(
        host=configs['robot']['ros_bridge']['host'],
        port=configs['robot']['ros_bridge']['port']
    )
    # ros_bridge.connect() # Uncomment when robot is ready
    
    # 6. Visualization
    # Prioritise projector.yaml if available, else fallback to arena.yaml
    if 'projector' in configs and 'projector' in configs['projector']:
        proj_conf = configs['projector']['projector']
        disp_conf = configs['projector'].get('display', {})
    else:
        proj_conf = configs['arena']['projector']
        disp_conf = configs['arena'].get('display', {})

    renderer = PygameRenderer(
        width=proj_conf['width'],
        height=proj_conf['height'],
        margin=proj_conf.get('margin_px', 50),
        fullscreen=disp_conf.get('fullscreen', False),
        display_index=disp_conf.get('display_index', 0)
    )
    renderer.set_arena_dimensions(
        configs['arena']['arena']['width_m'],
        configs['arena']['arena']['height_m']
    )
    
    # Variables de contrôle
    joystick = None
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"[MAIN] Joystick détecté: {joystick.get_name()}")
    
    # --- GAME LOOP ---
    try:
        clock = pygame.time.Clock()
        running = True
        
        # État initial du jeu
        game_state = {'status': 'ready'}
        tick_counter = 0

        while running:
            tick_counter += 1
            
            # 0. Events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    # Gérer ESC ici aussi si le renderer ne le capture pas
                    running = False
                
                # Le renderer gère ses propres events (ex: toggle debug 'D')
                renderer.handle_keypress(event)
            
            # Input Humain (Manette ou Clavier)
            human_input = {'v': 0.0, 'omega': 0.0, 'fire_request': False, 'start_game': False}
            
            max_linear_vel = configs['robot']['velocity_limits']['max_linear_mps']
            max_angular_vel = configs['robot']['velocity_limits']['max_angular_radps']
            
            # Joystick Input (PS4/Xbox)
            if joystick:
                v = -joystick.get_axis(1) * max_linear_vel
                omega = -joystick.get_axis(3) * max_angular_vel
                
                if abs(v) > 0.05:
                    human_input['v'] = v
                if abs(omega) > 0.05:
                    human_input['omega'] = omega
                
                if joystick.get_button(5) or joystick.get_button(0):
                    human_input['fire_request'] = True
                if joystick.get_button(9):
                    human_input['start_game'] = True

            # Keyboard fallback for movement (Arrow keys)
            keys = pygame.key.get_pressed()
            if keys[pygame.K_UP]:
                human_input['v'] = max_linear_vel
            if keys[pygame.K_DOWN]:
                human_input['v'] = -max_linear_vel
            if keys[pygame.K_LEFT]:
                human_input['omega'] = max_angular_vel
            if keys[pygame.K_RIGHT]:
                human_input['omega'] = -max_angular_vel
            if keys[pygame.K_SPACE]:
                human_input['fire_request'] = True
            if keys[pygame.K_RETURN]:
                human_input['start_game'] = True

            
            # 1. Vision
            color_frame, _ = camera.get_frames()
            if color_frame is None:
                continue
                
            detections = aruco.detect(color_frame)
            
            # 2. Update robot poses with proper coordinate transform (ANGLE CORRIGÉ)
            if 4 in detections:  # AI robot
                x, y, theta = get_robot_pose_in_world(detections[4], transform_mgr, fallback_scale)
                
                # FIX: Initialiser Kalman à la première détection (évite téléportation depuis 0,0)
                if np.all(kalman_ai.state == 0):
                    kalman_ai.state = np.array([x, y, 0, 0, theta, 0], dtype=float)
                
                kalman_ai.predict()
                kalman_ai.update((x, y, theta))
                filtered_pose = kalman_ai.get_pose()
                world.update_robot_pose(4, filtered_pose)
                 
            if 5 in detections:  # Human robot
                x, y, theta = get_robot_pose_in_world(detections[5], transform_mgr, fallback_scale)
                
                # FIX: Initialiser Kalman à la première détection (évite téléportation depuis 0,0)
                if np.all(kalman_human.state == 0):
                    kalman_human.state = np.array([x, y, 0, 0, theta, 0], dtype=float)
                
                kalman_human.predict()
                kalman_human.update((x, y, theta))
                filtered_pose = kalman_human.get_pose()
                world.update_robot_pose(5, filtered_pose)
            
            # 3. Update world occupancy
            world.update_occupancy()
            
            # 4. Game tick FIRST - to get current game status
            ai_decision = {'state': 'IDLE', 'fire_request': False, 'has_los': False, 'target_position': None}
            game_state = game_engine.tick(world, ai_decision, human_input)
            
            # Sync renderer state with game engine
            if game_state['state'] != renderer.match_state:
                if game_state['state'] == 'WAITING':
                    renderer.match_state = renderer.STATE_WAITING
                elif game_state['state'] == 'PLAYING':
                     # engine handles countdown implicitly, renderer handles visual countdown
                    if renderer.match_state != renderer.STATE_PLAYING:
                        renderer.start_match()
                elif game_state['state'] == 'FINISHED':
                    renderer.end_match(game_state.get('winner', 'NONE'))
            
            # Gestion du start game via renderer visual only
            if human_input['start_game']:
                 game_engine.start_match(duration_s=180) 
            
            # 5. Stratégie IA (si jeu en cours)
            if renderer.match_state == renderer.STATE_PLAYING:
                # Construit le contexte pour l'IA
                ai_context = world.get_state_dict()
                ai_context['ai_pose'] = world.get_robot_pose(4)
                ai_context['human_pose'] = world.get_robot_pose(5) # Target
                ai_context['raycast_sys'] = game_engine.raycast # Pour LOS
                
                ai_decision = ai_strategy.decide(ai_context)
                
                # Mise à jour debug renderer
                renderer.set_ai_path(ai_decision.get('path', []))
                
                # 6. Contrôle Robot (ROS)
                
                # CAS 1: DEPLACEMENT
                if ai_decision.get('target_position'):
                    # Calculer via contrôleur de trajectoire
                     # Note: ici on simplifie, idéalement on suit le path complet
                    path = ai_strategy.current_path
                    if path:
                        v_cmd, omega_cmd = controller.compute_control(world.get_robot_pose(4), path)
                        ros_bridge.send_velocity_command(4, v_cmd, omega_cmd)
                    else:
                        ros_bridge.send_velocity_command(4, 0, 0)
                    
                # CAS 2: TIR (Rotation sur place)
                elif ai_decision.get('fire_request'):
                    # Tourner vers la cible
                    enemy_pos = ai_decision.get('target_orientation')
                    if enemy_pos:
                         ai_pose = world.get_robot_pose(4)
                         dx = enemy_pos[0] - ai_pose[0]
                         dy = enemy_pos[1] - ai_pose[1]
                         desired_theta = np.arctan2(dy, dx)
                         
                         error_theta = desired_theta - ai_pose[2]
                         # Normalisation angle
                         while error_theta > np.pi: error_theta -= 2*np.pi
                         while error_theta < -np.pi: error_theta += 2*np.pi
                         
                         omega_cmd = np.clip(2.0 * error_theta, -max_angular_vel, max_angular_vel)
                         if abs(error_theta) < 0.08: omega_cmd = 0.0
                         
                         ros_bridge.send_velocity_command(4, 0.0, omega_cmd)
                    
                    # Demande de tir au ROS (si aligné ?)
                    # ros_bridge.send_fire_command()
                
                else:
                    # Stop si rien à faire
                    ros_bridge.send_velocity_command(4, 0, 0)

            # 7. Rendu Final
            renderer.render_frame(world.get_state_dict(), game_state)
            
            # Limite FPS
            clock.tick(30)
            
    except KeyboardInterrupt:
        print("[MAIN] Arrêt demandé par utilisateur.")
    except Exception as e:
        print(f"[MAIN] Erreur critique: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("[MAIN] Fermeture...")
        camera.stop()
        ros_bridge.disconnect()
        renderer.cleanup()
        pygame.quit()

if __name__ == '__main__':
    main()
