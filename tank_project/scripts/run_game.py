#!/usr/bin/env python3
"""
Script Principal du Jeu - Tank Arena

Intègre tous les composants :
- Perception (Caméra, ArUco, Kalman)
- Core (Moteur de Jeu, Modèle du Monde, IA)
- Visualisation (Projecteur Pygame)
- Contrôle (Pont ROS)

Usage:
    python3 run_game.py
"""

import os
import sys
import time
import yaml
import pygame
import numpy as np
import cv2  # Import nécessaire pour undistort
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.camera.realsense_stream import RealSenseStream
from perception.camera.aruco_detector import ArucoDetector
from perception.camera.kalman_filter import KalmanFilter
from core.world.world_model import WorldModel
from core.world.coordinate_frames import TransformManager
from core.world.unified_transform import UnifiedTransform, load_calibration
from core.game.game_engine import GameEngine
from core.game.state import GameStatus
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
    Configure la transformation de coordonnées.
    Priorité: calibration.json (nouveau) > arena.yaml (ancien)
    """
    # Essayer d'abord le nouveau système (calibration.json)
    config_dir = Path(__file__).parent.parent / 'config'
    unified = load_calibration(str(config_dir))
    
    if unified.is_calibrated():
        print("[MAIN] ✓ Calibration JSON chargée (UnifiedTransform)")
        return unified
    
    # Fallback: ancien système YAML
    print("[MAIN] Fallback vers ancien système (TransformManager)")
    transform_mgr = TransformManager()
    
    if 'transform' in configs['arena'] and 'H_C2W' in configs['arena']['transform']:
        transform_mgr.H_C2W = np.array(configs['arena']['transform']['H_C2W'])
        print("[MAIN] Matrice d'homographie YAML chargée.")
    else:
        print("[MAIN] ATTENTION : Pas de calibration trouvée!")
        print("[MAIN] Lancez: python -m perception.calibration.standalone_wizard")
        if 'scale' in configs['arena'].get('transform', {}):
             transform_mgr.set_av_to_world_scale(configs['arena']['transform']['scale'])
    
    return transform_mgr


def camera_to_world(detection_center, transform_mgr, fallback_scale=1.0):
    """
    Transform ArUco detection from camera pixels to world meters.
    Supporte UnifiedTransform (nouveau) et TransformManager (ancien).
    
    Args:
        detection_center: (u, v) pixel coordinates
        transform_mgr: UnifiedTransform ou TransformManager instance
        fallback_scale: Scale to use if no homography
        
    Returns:
        (x, y) in meters
    """
    u, v = detection_center
    
    # Nouveau système (UnifiedTransform)
    if isinstance(transform_mgr, UnifiedTransform):
        if transform_mgr.is_calibrated():
            return transform_mgr.camera_to_world(u, v)
    # Ancien système (TransformManager)
    elif hasattr(transform_mgr, 'H_C2W') and transform_mgr.H_C2W is not None:
        return transform_mgr.camera_to_world(u, v)
    
    # Fallback: simple scaling
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
    print("[MAIN] ========== Jeu Tank Arena ==========")
    
    # 1. Configuration
    configs = load_config()
    
    # Init subsystems
    transform_mgr = setup_transform_manager(configs)
    
    fallback_scale = 1.0
    if 'transform' in configs['arena'] and 'scale' in configs['arena']['transform']:
        fallback_scale = configs['arena']['transform']['scale']

    # 1. Perception
    camera = RealSenseStream(
        width=configs['camera']['realsense'].get('width', 1280),
        height=configs['camera']['realsense'].get('height', 720),
        fps=configs['camera']['realsense'].get('fps', 30)
    )
    camera.start()
    
    # Attente pour l'initialisation du pipeline (Recommandé)
    time.sleep(1.0)
    
    # --- CORRECTION DISTORSION ---
    # Récupération des paramètres intrinsèques
    K, D = camera.get_intrinsics_matrix()
    if K is not None:
        print("[MAIN] Paramètres optiques chargés (suppression distorsion activée)")
    else:
        print("[MAIN] ATTENTION: Impossible de charger les paramètres optiques")
    
    aruco = ArucoDetector()
    kalman_ai = KalmanFilter()
    kalman_human = KalmanFilter()
    
    # 2. World - avec config robot
    # Priorité: dimensions de calibration JSON > arena.yaml
    robot_config = configs['arena'].get('robot', {})
    
    if isinstance(transform_mgr, UnifiedTransform) and transform_mgr.is_calibrated():
        arena_w = transform_mgr.arena_width_m
        arena_h = transform_mgr.arena_height_m
        print(f"[MAIN] Dimensions arène depuis calibration: {arena_w:.2f}x{arena_h:.2f}m")
    else:
        arena_w = configs['arena']['arena']['width_m']
        arena_h = configs['arena']['arena']['height_m']
        print(f"[MAIN] Dimensions arène depuis YAML: {arena_w:.2f}x{arena_h:.2f}m")
    
    world = WorldModel(
        arena_width_m=arena_w,
        arena_height_m=arena_h,
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
    # Fusionner les limites de vitesse dans la config de contrôle
    control_conf = configs['robot'].get('control', {}).copy()
    if 'velocity_limits' in configs['robot']:
        control_conf.update(configs['robot']['velocity_limits'])
        
    controller = TrajectoryFollower(control_conf)
    ros_bridge = ROSBridgeClient(
        host=configs['robot']['ros_bridge']['host'],
        port=configs['robot']['ros_bridge']['port']
    )
    # Tentative de connexion au robot (continue même si échec)
    if not ros_bridge.connect(max_retries=2, retry_interval=1.0):
        print("[MAIN] ATTENTION: Robot non connecté, mode simulation uniquement")
    
    # 6. Visualization
    # Configuration depuis projector.yaml
    if 'projector' in configs and 'projector' in configs['projector']:
        proj_conf = configs['projector']['projector']
        disp_conf = configs['projector'].get('display', {})
    else:
        # Fallback par défaut
        print("[MAIN] ATTENTION: projector.yaml non trouvé, utilisation des valeurs par défaut")
        proj_conf = {'width': 1024, 'height': 768}
        disp_conf = {'fullscreen': False, 'monitor_offset_x': 0, 'monitor_offset_y': 0, 'borderless': True}

    # Setup window position manually before init
    monitor_x = disp_conf.get('monitor_offset_x', 0)
    monitor_y = disp_conf.get('monitor_offset_y', 0)
    os.environ['SDL_VIDEO_WINDOW_POS'] = f"{monitor_x},{monitor_y}"

    renderer = PygameRenderer(
        width=proj_conf['width'],
        height=proj_conf['height'],
        margin=proj_conf.get('margin_px', 50),
        monitor_offset_x=disp_conf.get('monitor_offset_x', 0),
        monitor_offset_y=disp_conf.get('monitor_offset_y', 0),
        borderless=disp_conf.get('borderless', True),
        transform_mgr=transform_mgr
    )
    renderer.set_arena_dimensions(arena_w, arena_h) # Fallback init
    
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
        
        # Timing dynamique pour Kalman (au lieu de dt codé en dur)
        last_time = time.time()

        while running:
            tick_counter += 1
            
            # Calcul du vrai dt (temps écoulé depuis la dernière itération)
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            # Limiter dt pour éviter les sauts après pause/lag (max 100ms)
            dt = min(dt, 0.1)
            
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
            
            # --- APPLICATION CORRECTION DISTORSION ---
            if K is not None and D is not None:
                color_frame = cv2.undistort(color_frame, K, D)
                
            detections = aruco.detect(color_frame)
            
            # --- CORRECTION PARALLAXE (3D) ---
            # DÉSACTIVÉ : show_grid.py fonctionne mieux sans correction (homographie suffit)
            # if K is not None:
            #     # L'image est déjà corrigée de la distorsion (undistort plus haut),
            #     # donc on utilise D=0 pour le calcul de pose PnP
            #     D_pnp = np.zeros(5)
            #     
            #     for rid in [4, 5]:
            #         if rid in detections:
            #             raw = detections[rid]['corners']
            #             (u_c, v_c), _ = aruco.get_corrected_pose(
            #                 np.array(raw, dtype=np.float32), 
            #                 K, D_pnp, obj_height_m=0.30
            #             )
            #             detections[rid]['center'] = (u_c, v_c)
            
            # 2. Update robot poses with proper coordinate transform (ANGLE CORRIGÉ)
            # Compteurs pour détecter les pertes de tracking prolongées
            if not hasattr(main, 'ai_lost_frames'):
                main.ai_lost_frames = 0
                main.human_lost_frames = 0
            
            if 4 in detections:  # AI robot détecté
                main.ai_lost_frames = 0
                x, y, theta = get_robot_pose_in_world(detections[4], transform_mgr, fallback_scale)
                
                # FIX 1: CORRECTION ORIENTATION (Inversion 180°)
                # Le robot physique a l'ArUco inversé par rapport à la logique d'avancée
                theta = (theta + np.pi) % (2 * np.pi)
                
                # Garde-fou : On force x et y à rester dans l'arène
                # On laisse une marge de 5cm (0.05) pour ne pas être DANS le mur
                x = max(0.05, min(x, arena_w - 0.05))
                y = max(0.05, min(y, arena_h - 0.05))
                
                # FIX: Initialiser Kalman à la première détection (évite téléportation depuis 0,0)
                if np.all(kalman_ai.state == 0):
                    kalman_ai.state = np.array([x, y, 0, 0, theta, 0], dtype=float)
                
                kalman_ai.predict(dt)  # Utilise le dt dynamique
                kalman_ai.update((x, y, theta))
                filtered_pose = kalman_ai.get_pose()
                world.update_robot_pose(4, filtered_pose)
            else:
                # Robot IA non détecté - continue avec prédiction Kalman
                main.ai_lost_frames += 1
                if main.ai_lost_frames == 30:  # ~1 seconde
                    print("[MAIN] ATTENTION: Robot IA (ID 4) non détecté depuis 1s")
                
                # Mise à jour Kalman avec le VRAI dt
                kalman_ai.dt = dt
                kalman_ai.predict()  # Utilise le dt interne
                
                # FIX 1 (Pathfinding): Clamp prediction pour éviter coordonnées hors limites
                pred_pose = list(kalman_ai.get_pose())  # Convertir en liste pour pouvoir modifier
                pred_pose[0] = max(0.05, min(pred_pose[0], arena_w - 0.05))
                pred_pose[1] = max(0.05, min(pred_pose[1], arena_h - 0.05))
                
                world.update_robot_pose(4, tuple(pred_pose))
                 
            if 5 in detections:  # Human robot détecté
                main.human_lost_frames = 0
                x, y, theta = get_robot_pose_in_world(detections[5], transform_mgr, fallback_scale)
                
                # Garde-fou : On force x et y à rester dans l'arène
                x = max(0.05, min(x, arena_w - 0.05))
                y = max(0.05, min(y, arena_h - 0.05))
                
                # FIX: Initialiser Kalman à la première détection (évite téléportation depuis 0,0)
                if np.all(kalman_human.state == 0):
                    kalman_human.state = np.array([x, y, 0, 0, theta, 0], dtype=float)
                
                kalman_human.predict(dt)  # Utilise le dt dynamique
                kalman_human.update((x, y, theta))
                filtered_pose = kalman_human.get_pose()
                world.update_robot_pose(5, filtered_pose)
            else:
                # Robot Humain non détecté - continue avec prédiction Kalman
                main.human_lost_frames += 1
                if main.human_lost_frames == 30:  # ~1 seconde
                    print("[MAIN] ATTENTION: Robot Humain (ID 5) non détecté depuis 1s")
                
                # Mise à jour Kalman avec le VRAI dt
                kalman_human.dt = dt
                kalman_human.predict()
                
                # FIX 1 (Pathfinding): Clamp prediction pour éviter coordonnées hors limites
                pred_pose = list(kalman_human.get_pose())  # Convertir en liste pour pouvoir modifier
                pred_pose[0] = max(0.05, min(pred_pose[0], arena_w - 0.05))
                pred_pose[1] = max(0.05, min(pred_pose[1], arena_h - 0.05))
                
                world.update_robot_pose(5, tuple(pred_pose))
            
            # 3. Update world occupancy
            world.update_occupancy()
            
            # 4. Stratégie IA (Calcul AVANT le tick moteur pour prise en compte des actions)
            # Init default decision to avoid unbound variable
            ai_decision = {'state': 'IDLE', 'fire_request': False, 'has_los': False, 'target_position': None}
            
            # L'IA démarre seulement si le mode de jeu est actif (Visual Timer fini)
            if renderer.match_state == renderer.STATE_PLAYING:
                # Construit le contexte pour l'IA
                ai_context = world.get_state_dict()
                ai_context['ai_pose'] = world.get_robot_pose(4)
                ai_context['human_pose'] = world.get_robot_pose(5) # Target
                ai_context['raycast_sys'] = game_engine.raycast # Pour LOS
                
                ai_decision = ai_strategy.decide(ai_context)
                
                # Mise à jour debug renderer (chemin)
                renderer.set_ai_path(ai_strategy.get_full_path())
            
            # 5. Game tick (Traite les tirs et la logique de jeu GLOBALE)
            game_state = game_engine.tick(world, ai_decision, human_input)
            
            # 6. FIX 2 & 3: Visualisation des Tirs et Gestion Score "Simple"
            from core.game.hits import HitEvent # Nécessaire pour l'injection manuelle
            # import time removed (global import used)
            current_t = time.time()
            
            # Tir Joueur (Espace) -> Robot 5
            # On vérifie si un tir a été enregistré RÉCEMMENT (dans ce tick) par le moteur
            if human_input.get('fire_request', False) and (current_t - game_engine.cooldowns.last_shot_human < 0.2):
                p5 = world.get_robot_pose(5)
                p4 = world.get_robot_pose(4)
                
                if p5 and p4:
                    # 1. Visualisation
                    shot_res = game_engine.raycast.cast_shot((p5[0], p5[1]), p5[2], max_range_m=5.0)
                    dist = shot_res['distance']
                    renderer.add_shot_fx(p5, dist, color=(255, 255, 0)) # JAUNE
                    
                    # 2. Logique de touche "Distance Simple"
                    dist_between_robots = np.linalg.norm(np.array(p5[:2]) - np.array(p4[:2]))
                    
                    # Si le rayon s'arrête environ à la distance du robot (donc il l'a touché sur la grille)
                    # OU si le rayon va plus loin mais passait par là (simulation simple)
                    # SEUIL RÉDUIT à 0.15m (15cm) comme demandé
                    
                    is_hit = abs(dist - dist_between_robots) < 0.15
                    
                    if is_hit:
                         # Mise à jour score local immédiat pour affichage
                         game_state['robot_5_hits_inflicted'] = game_state.get('robot_5_hits_inflicted', 0) + 1
                         print(f"[GAME] HIT CONFIRMÉ ! Joueur touche IA. Score: {game_state['robot_5_hits_inflicted']}")
                         
                         # Injection manuelle dans l'engine pour persistence
                         event = HitEvent(shooter_id=5, target_id=4, impact_point=p4[:2], timestamp=time.time())
                         game_engine.hit_manager.hit_history.append(event)

            # Tir IA -> Robot 4
            # On vérifie si un tir a été enregistré RÉCEMMENT (dans ce tick) par le moteur
            if ai_decision.get('fire_request', False) and (current_t - game_engine.cooldowns.last_shot_ai < 0.2):
                p4 = world.get_robot_pose(4)
                p5 = world.get_robot_pose(5)
                
                if p4 and p5:
                    # 1. Visualisation
                    shot_res = game_engine.raycast.cast_shot((p4[0], p4[1]), p4[2], max_range_m=5.0)
                    dist = shot_res['distance']
                    renderer.add_shot_fx(p4, dist, color=(255, 0, 0)) # ROUGE
                    
                    # 2. Logique de touche "Distance Simple"
                    dist_between_robots = np.linalg.norm(np.array(p4[:2]) - np.array(p5[:2]))
                    is_hit = abs(dist - dist_between_robots) < 0.15
                    
                    if is_hit:
                        # Mise à jour score local immédiat
                        game_state['robot_4_hits_inflicted'] = game_state.get('robot_4_hits_inflicted', 0) + 1
                        print(f"[GAME] HIT CONFIRMÉ ! IA touche Joueur. Score: {game_state['robot_4_hits_inflicted']}")
                        
                        # Injection manuelle
                        event = HitEvent(shooter_id=4, target_id=5, impact_point=p5[:2], timestamp=time.time())
                        game_engine.hit_manager.hit_history.append(event)
            
            # 7. Sync renderer state
            current_status = str(game_state.get('status', 'ready')).lower()
            
            if current_status == 'ready':
                renderer.match_state = renderer.STATE_WAITING
            elif current_status == 'playing':
                 # engine handles countdown implicitly, renderer handles visual countdown
                if renderer.match_state == renderer.STATE_WAITING:
                     renderer.start_match()
                # On ne force pas le PLAYING tant que le renderer est en COUNTDOWN
            elif current_status == 'finished':
                renderer.end_match(game_state.get('winner', 'NONE'))
            
            # Start game input
            if human_input['start_game'] and game_engine.state == GameStatus.READY:
                 game_engine.start_match(duration_s=180) 

            # 8. Contrôle Robot (ROS)
            if renderer.match_state == renderer.STATE_PLAYING:
                # CAS 1: DEPLACEMENT
                if ai_decision.get('target_position'):
                    # Calculer via contrôleur de trajectoire
                    path = ai_strategy.current_path
                    # SÉCURITÉ: Vérifier que le chemin existe ET n'est pas vide
                    if path and len(path) > 0:
                        ai_pose = world.get_robot_pose(4)
                        
                        # SEUIL DE PROXIMITÉ: Si trop proche du waypoint (5cm), avancer au suivant
                        next_wp = path[0]
                        dist_to_wp = np.linalg.norm([ai_pose[0] - next_wp[0], ai_pose[1] - next_wp[1]])
                        
                        # FIX CRITIQUE: Arrêt physique si distance < 5cm
                        # Cela empêche les oscillations frénétiques et le spin en fin de trajectoire
                        if dist_to_wp < 0.05:  # 5cm threshold
                            # Force arrêt complet
                            ros_bridge.send_velocity_command(4, 0.0, 0.0)
                            # Avance au waypoint suivant
                            ai_strategy.advance_waypoint()
                            path = ai_strategy.current_path
                            
                            # Si plus de waypoints, continuer l'arrêt
                            if not path or len(path) == 0:
                                # Déjà envoyé stop ci-dessus, pas besoin de répéter
                                pass
                        else:
                            # Distance normale, calculer les commandes
                            v_cmd, omega_cmd = controller.compute_control(ai_pose, path)
                            # DEBUG: Log pose utilisée et commandes calculées (tous les 30 ticks)
                            if tick_counter % 30 == 0:
                                print(f"[DEBUG] ArUco4 détecté: {4 in detections}, Pose IA: ({ai_pose[0]:.3f}, {ai_pose[1]:.3f}, {ai_pose[2]:.2f})")
                                print(f"[DEBUG] Path[0]: {path[0] if path else 'None'}, dist={dist_to_wp:.3f}m, v={v_cmd:.3f}, w={omega_cmd:.3f}")
                            ros_bridge.send_velocity_command(4, v_cmd, omega_cmd)
                    else:
                        ros_bridge.send_velocity_command(4, 0, 0)
                    
                # CAS 2: ROTATION (Tir ou Visée)
                elif ai_decision.get('target_orientation') is not None:
                    # Tourner vers la cible
                    enemy_pos = ai_decision.get('target_orientation')
                    rotation_done = False
                    
                    if enemy_pos is not None:
                         # Extraction robuste des coordonnées (supporte list, tuple, numpy)
                         try:
                             # Vérifie si indexable sans utiliser isinstance restrictif
                             ex = enemy_pos[0]
                             ey = enemy_pos[1]
                                 
                             ai_pose = world.get_robot_pose(4)
                             dx = ex - ai_pose[0]
                             dy = ey - ai_pose[1]
                             desired_theta = np.arctan2(dy, dx)
                             
                             error_theta = desired_theta - ai_pose[2]
                             # Normalisation angle
                             while error_theta > np.pi: error_theta -= 2*np.pi
                             while error_theta < -np.pi: error_theta += 2*np.pi
                             
                             omega_cmd = np.clip(2.0 * error_theta, -max_angular_vel, max_angular_vel)
                             
                             # Deadband pour éviter l'oscillation
                             if abs(error_theta) < 0.08: omega_cmd = 0.0
                             
                             ros_bridge.send_velocity_command(4, 0.0, omega_cmd)
                             rotation_done = True
                         except Exception as e:
                             print(f"[AI] Erreur rotation tir: {e}")
                    
                    if not rotation_done:
                        ros_bridge.send_velocity_command(4, 0, 0)
                else:
                    # Rien à faire, stop
                    ros_bridge.send_velocity_command(4, 0, 0)

            # 7. Rendu Final
            
            # Préparation de l'image de fond (AR)
            bg_image = None
            if color_frame is not None and hasattr(transform_mgr, 'H_CamToProj'):
                # WARPING MAGIQUE : On transforme l'image caméra vers la perspective du projecteur
                if transform_mgr.H_CamToProj is not None:
                    try:
                        bg_image = cv2.warpPerspective(
                            color_frame, 
                            transform_mgr.H_CamToProj, 
                            (proj_conf['width'], proj_conf['height'])
                        )
                    except Exception:
                        pass
                else:
                    bg_image = color_frame # Fallback

            renderer.render_frame(world.get_state_dict(), game_state, background_image=bg_image)
            
            # Limite FPS
            # On monte à 60 pour ne pas brider la caméra (qui bloque à 30fps)
            # Cela permet de traiter la frame dès qu'elle arrive
            clock.tick(60)
            
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
