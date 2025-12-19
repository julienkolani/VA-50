#!/usr/bin/env python3
"""
Main Game Loop

Runs the tank arena game at 30 FPS:
1. Vision and tracking (ArUco detection, Kalman filtering)
2. World update (occupancy grid, robot poses)
3. Game engine tick (shots, hits, timers)
4. AI decision (behavior tree, path planning)
5. Control (trajectory following, ROS commands)
6. Visualization (Pygame rendering)

Usage:
    python3 run_game.py
    
Prerequisites:
    - Calibration completed (config/arena.yaml exists)
    - ROS bridge running
    - RealSense camera connected
    - Projector configured
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
from perception.camera.homography import apply_homography_single
from core.world.world_model import WorldModel
from core.world.coordinate_frames import TransformManager
from core.game.game_engine import GameEngine
from core.ia.strategy import AIStrategy
from core.control.trajectory_follower import TrajectoryFollower
from core.control.ros_bridge_client import ROSBridgeClient
from visualization.pygame_renderer import PygameRenderer


def load_config():
    """Load all configuration files."""
    config_dir = Path(__file__).parent.parent / 'config'
    
    configs = {}
    for config_file in ['arena', 'camera', 'game', 'ia', 'robot']:
        with open(config_dir / '{}.yaml'.format(config_file)) as f:
            configs[config_file] = yaml.safe_load(f)
    
    return configs


def setup_transform_manager(configs):
    """
    Setup coordinate transform from calibration data.
    
    Returns:
        TransformManager with H_C2W set, or None if not calibrated
    """
    transform_mgr = TransformManager()
    
    # Load H_C2W from calibration if available
    transform_data = configs['arena'].get('transform', {})
    scale = transform_data.get('scale_m_per_av', 1.0)
    h_c2w = transform_data.get('H_C2W')
    
    if h_c2w is not None:
        transform_mgr.H_C2W = np.array(h_c2w)
        print("[MAIN] Loaded H_C2W from calibration")
    else:
        # Fallback: use simple scaling (assumes camera aligned with arena)
        transform_mgr.set_av_to_world_scale(scale)
        print("[MAIN] WARNING: No H_C2W found, using fallback scaling")
    
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
    
    # Load configuration
    print("[MAIN] Loading configuration...")
    configs = load_config()
    
    # Setup coordinate transforms
    transform_mgr = setup_transform_manager(configs)
    
    # Initialize subsystems
    print("[MAIN] Initializing subsystems...")
    
    # 1. Vision
    camera = RealSenseStream(
        width=configs['camera']['realsense']['width'],
        height=configs['camera']['realsense']['height'],
        fps=configs['camera']['realsense']['fps']
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
    
    # 5. Control - get velocity limits from correct config path
    velocity_limits = configs['robot'].get('velocity_limits', {})
    max_linear_vel = velocity_limits.get('max_linear_mps', 0.22)
    max_angular_vel = velocity_limits.get('max_angular_radps', 2.84)
    
    controller = TrajectoryFollower(configs['robot'].get('control', {}))
    ros_bridge = ROSBridgeClient(
        host=configs['robot']['ros_bridge']['host'],
        port=configs['robot']['ros_bridge']['port']
    )
    ros_bridge.connect()
    
    # 6. Visualization
    display_config = configs['arena'].get('display', {})
    renderer = PygameRenderer(
        width=configs['arena']['projector']['width'],
        height=configs['arena']['projector']['height'],
        margin=configs['arena']['projector']['margin_px'],
        fullscreen=display_config.get('fullscreen', False),
        display_index=display_config.get('display_index', 0)
    )
    renderer.set_arena_dimensions(
        configs['arena']['arena']['width_m'],
        configs['arena']['arena']['height_m']
    )
    
    print("[MAIN] All subsystems initialized")
    print("[MAIN] Starting game loop at 30 FPS...")
    
    # Initialize Input (Joystick/Keyboard)
    joystick = None
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print("[MAIN] Joystick detected: {}".format(joystick.get_name()))
    
    # Game loop
    dt = 1.0 / configs['game']['match']['tick_rate_fps']
    running = True
    
    # Fallback scale from config
    fallback_scale = configs['arena'].get('transform', {}).get('scale_m_per_av', 1.0)
    
    # Initial game state - used to check if playing before game_engine.tick() is called
    game_state = {'status': 'ready'}
    tick_counter = 0
    
    print("[LOG] === GAME LOOP STARTING ===")
    print(f"[LOG] Initial game_state: {game_state}")
    print(f"[LOG] Renderer state: {renderer.match_state}")
    
    try:
        while running:
            tick_start = time.time()
            tick_counter += 1
            
            # --- INPUT HANDLING ---
            human_input = {
                'v': 0.0,
                'omega': 0.0,
                'fire_request': False,
                'start_game': False
            }
            
            # Process Pygame Events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    print("[LOG] QUIT event received")
                elif event.type == pygame.VIDEORESIZE:
                    # Handle window resize
                    renderer.handle_resize(event.w, event.h)
                elif event.type == pygame.KEYDOWN:
                    print(f"[LOG] KEYDOWN: {pygame.key.name(event.key)}")
                    if event.key == pygame.K_ESCAPE:
                        if renderer.fullscreen:
                            renderer.toggle_fullscreen()
                        else:
                            running = False
                    elif event.key == pygame.K_F11:
                        renderer.toggle_fullscreen()
                    elif event.key == pygame.K_SPACE:
                        human_input['fire_request'] = True
                        if renderer.match_state == renderer.STATE_WAITING:
                            human_input['start_game'] = True
                            print("[LOG] SPACE pressed -> start_game = True")
                    elif event.key == pygame.K_RETURN:
                        human_input['start_game'] = True
                        print("[LOG] RETURN pressed -> start_game = True")
                    elif event.key == pygame.K_d:
                        renderer.show_debug_path = not renderer.show_debug_path
                        print(f"[LOG] Debug path: {'ON' if renderer.show_debug_path else 'OFF'}")
            
            # Sync renderer state with game engine start
            if human_input['start_game']:
                print(f"[LOG] start_game=True, renderer.match_state={renderer.match_state}")
                if renderer.match_state == renderer.STATE_WAITING:
                    renderer.start_match(countdown_seconds=3.0)
                    print("[LOG] >>> MATCH COUNTDOWN STARTED <<<")
                         
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
            
            # Log state changes
            if tick_counter % 30 == 0:  # Every second
                print(f"[LOG] Tick {tick_counter}: game_state.status={game_state.get('status')}, renderer={renderer.match_state}")
                print(f"[LOG]   Robot4: {world.get_robot_pose(4)}, Robot5: {world.get_robot_pose(5)}")
            
            # 5. AI Decision (only if game is playing)
            if game_state.get('status', 'ready') == 'playing':
                if tick_counter % 30 == 0:
                    print("[LOG] >>> GAME IS PLAYING - AI DECIDING <<<")
                
                # Construct AI Context
                ai_context = world.get_state_dict()
                ai_context['ai_pose'] = world.get_robot_pose(4)
                ai_context['human_pose'] = world.get_robot_pose(5)
                ai_context['raycast_sys'] = game_engine.raycast
                ai_context['game_time'] = time.time()
                
                ai_decision = ai_strategy.decide(ai_context)
                
                if tick_counter % 30 == 0:
                    print(f"[LOG]   AI decision: state={ai_decision.get('state')}, target={ai_decision.get('target_position')}")
                    print(f"[LOG]   AI path length: {len(ai_strategy.current_path)}")
            
            # 6. Control (only if game is playing)
            if game_state.get('status', 'ready') == 'playing':
                # AI Control
                if ai_decision.get('target_position'):
                    path = ai_strategy.current_path
                    if path:
                        v_ai, omega_ai = controller.compute_control(world.get_robot_pose(4), path)
                        ros_bridge.send_velocity_command(4, v_ai, omega_ai)
                        renderer.set_ai_path(path)
                        
                        if tick_counter % 30 == 0:
                            print(f"[LOG]   Sending cmd: v={v_ai:.3f}, omega={omega_ai:.3f}")
                    else:
                        ros_bridge.send_velocity_command(4, 0, 0)
                        if tick_counter % 30 == 0:
                            print("[LOG]   No path - sending STOP")
                else:
                    ros_bridge.send_velocity_command(4, 0, 0)
                    if tick_counter % 30 == 0:
                        print("[LOG]   No target_position - sending STOP")
                     
                # Human Control (for RC mode)
                if human_input['v'] != 0 or human_input['omega'] != 0:
                    ros_bridge.send_velocity_command(5, human_input['v'], human_input['omega'])
            else:
                # Not playing - ensure robots are stopped
                ros_bridge.send_velocity_command(4, 0, 0)
                ros_bridge.send_velocity_command(5, 0, 0)

            
            # 7. Render
            renderer.render_frame(world.get_state_dict(), game_state)
            
            # Maintain frame rate
            elapsed = time.time() - tick_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
                
    except KeyboardInterrupt:
        print("\n[MAIN] Shutting down...")
        
    finally:
        # Cleanup
        camera.stop()
        ros_bridge.disconnect()
        renderer.cleanup()
        print("[MAIN] Goodbye!")


if __name__ == '__main__':
    main()
