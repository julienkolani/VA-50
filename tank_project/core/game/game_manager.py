#!/usr/bin/env python3
import sys
import time
import yaml
import numpy as np
import pygame
import math
import random
from pathlib import Path

# Add project root needed for imports if run directly
ROOT_DIR = Path(__file__).parent.parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

# Conditional Import for Mock support
try:
    import cv2
    from perception.camera.realsense_stream import RealSenseStream
    from perception.camera.aruco_detector import ArucoDetector
    PERCEPTION_AVAILABLE = True
except ImportError:
    print("[MANAGER] Warning: Perception modules (cv2/realsense) not found. Forcing MOCK mode.")
    PERCEPTION_AVAILABLE = False

from core.world.unified_transform import load_calibration
from core.ia.strategy import AIStrategy
from core.world.world_model import WorldModel
from renderer.game_renderer import GameRenderer
from core.control.ros_bridge_client import ROSBridgeClient

class GameManager:
    def __init__(self, mock=False):
        self.root_dir = ROOT_DIR
        self.config_dir = self.root_dir / 'config'
        
        # 1. Load Calibration & Configs
        self.tm = load_calibration(str(self.config_dir))
        self.configs = self._load_configs()
        
        # Game Configs
        game_cfg = self.configs.get('game', {})
        self.game_duration = game_cfg.get('duration', 180)
        
        shot_cfg = game_cfg.get('shooting', {})
        self.SHOT_COOLDOWN = shot_cfg.get('cooldown', 1.0)
        self.MAX_SHOT_DIST = shot_cfg.get('max_range', 10.0)
        self.HIT_RADIUS = shot_cfg.get('hit_radius', 0.25)
        
        # Mock Configs
        mock_cfg = game_cfg.get('mock', {})
        # Note: Config stores relative pos [x%, y%, theta]
        mock_ai = mock_cfg.get('ai_start_pos', [0.2, 0.5, 0.0])
        mock_hu = mock_cfg.get('human_start_pos', [0.8, 0.5, 3.14])
        
        self.mock_ai_pos = [
            self.tm.arena_width_m * mock_ai[0],
            self.tm.arena_height_m * mock_ai[1],
            mock_ai[2]
        ]
        self.mock_hu_pos = [
            self.tm.arena_width_m * mock_hu[0],
            self.tm.arena_height_m * mock_hu[1],
            mock_hu[2]
        ]
        
        # Mock override
        self.mock_mode = mock or (not PERCEPTION_AVAILABLE)
        
        # 2. Setup Components
        self.camera = None
        self.aruco = None
        self._setup_perception()
        
        self.world_model = None
        self._setup_world()
        
        self.ai = None
        self._setup_ai()
        
        self.renderer = None
        self._setup_renderer()
        
        self.ros_bridge = None
        self._setup_control()
        
        # 3. Game State
        self.status = "WAITING" 
        self.scores = {"ai": 0, "human": 0}
        self.start_time = 0
        self.last_shot_time = {"ai": 0, "human": 0}
        self.running = True

    # ... (existing methods _load_configs, _setup_control, _setup_perception, _setup_world, _setup_ai, _setup_renderer) ...

    def _attempt_fire(self, shooter_name, shooter_pose, target_pose):
        now = time.time()
        if now - self.last_shot_time.get(shooter_name, 0) < self.SHOT_COOLDOWN:
            return

        self.last_shot_time[shooter_name] = now
        
        # 1. Physics: Raycasting
        sx, sy, sth = shooter_pose
        cos_th = math.cos(sth)
        sin_th = math.sin(sth)
        
        # Default: Max Range
        dist_final = self.MAX_SHOT_DIST
        
        # Check Wall Intersection (Box Arena)
        candidates = []
        if abs(cos_th) > 0.001:
            t1 = (0 - sx) / cos_th; t2 = (self.arena_w - sx) / cos_th
            if t1 > 0: candidates.append(t1)
            if t2 > 0: candidates.append(t2)
        if abs(sin_th) > 0.001:
            t3 = (0 - sy) / sin_th; t4 = (self.arena_h - sy) / sin_th
            if t3 > 0: candidates.append(t3)
            if t4 > 0: candidates.append(t4)
            
        if candidates:
            dist_wall = min(candidates)
            dist_final = min(dist_wall, dist_final)
        
        end_x = sx + dist_final * cos_th
        end_y = sy + dist_final * sin_th
        
        # 2. Check Hit on Target (Circle Intersection)
        hit_confirmed = False
        # HIT_RADIUS is now loaded from config in __init__
        
        if target_pose:
            tx, ty, _ = target_pose
            # Vector Shooter->Target
            idx = tx - sx
            idy = ty - sy
            # Projection scalar
            proj = (idx * cos_th + idy * sin_th)
            
            # If target is in front and within range
            if 0 < proj < dist_final:
                # Closest point on line
                cpx = sx + proj * cos_th
                cpy = sy + proj * sin_th
                
                dist_sq = (cpx - tx)**2 + (cpy - ty)**2
                if dist_sq < (self.HIT_RADIUS**2):
                    hit_confirmed = True
                    # Snap shot end to impact point roughly
                    end_x, end_y = cpx, cpy

    def _load_configs(self):
        cfgs = {}
        for name in ['game', 'arena', 'robot']:
            p = self.config_dir / f"{name}.yaml"
            if p.exists():
                with open(p) as f: cfgs[name] = yaml.safe_load(f)
        return cfgs

    def _setup_control(self):
        if self.mock_mode: return
        self.ros_bridge = ROSBridgeClient(host='127.0.0.1', port=8765)
        if self.ros_bridge.connect():
            print("[MANAGER] ROS Bridge Connected.")
        else:
            print("[MANAGER] WARNING: Failed to connect to ROS Bridge.")

    def _setup_perception(self):
        if self.mock_mode:
            print("[MANAGER] Mock Mode: Perception disabled.")
            return

        if not PERCEPTION_AVAILABLE:
             print("[MANAGER] ERROR: Perception modules not installed. Cannot run in Real mode.")
             print("           Install requirements or run with --mock.")
             sys.exit(1)

        try:
            self.camera = RealSenseStream()
            self.camera.start()
            # ArUco Config (ID 4=AI, ID 5=Human for Tank Arena)
            self.aruco = ArucoDetector(dictionary_type=cv2.aruco.DICT_4X4_50)
            print("[MANAGER] Perception initialized (Realsense + ArUco).")
        except Exception as e:
            print(f"[MANAGER] CRITICAL ERROR: Failed to initialize Perception.")
            print(f"Details: {e}")
            print("Check camera connection. To run simulation, use --mock.")
            # Do NOT switch to mock automatically. User expects real data.
            raise e

    def _setup_world(self):
        # We assume simple rectangular arena for raycasting
        self.arena_w = self.tm.arena_width_m
        self.arena_h = self.tm.arena_height_m
        
        # Load World Model for AI usage
        # Note: WorldModel constructor arguments might need tuning based on config
        self.world_model = WorldModel(self.arena_w, self.arena_h)

    def _setup_ai(self):
        # Simple AI config
        ai_conf = self.configs.get('ia', {})
        self.ai = AIStrategy(ai_conf)
        self.ai.set_world_model(self.world_model)

    def _setup_renderer(self):
        self.renderer = GameRenderer(str(self.config_dir))

    # --- MAIN LOOP ---
    def run(self):
        print("[MANAGER] Game Manager Running. Controls: SPACE=Start/Stop AI, F=User Fire")
        
        try:
            while self.running and self.renderer.running:
                # 1. Input / System Events
                if not self.renderer.process_events():
                    self.running = False
                    break
                
                # 2. Perception (Always Active)
                ai_pose, human_pose = self._update_perception()
                
                if ai_pose: self.world_model.update_robot_pose(4, ai_pose)
                if human_pose: self.world_model.update_robot_pose(5, human_pose)
                
                # 3. Game Logic Inputs
                self._handle_input(human_pose)
                
                # 4. AI & Game Loop
                if self.status == "RUNNING":
                    # Time Check (Simple Duration)
                    if time.time() - self.start_time > self.game_duration:
                        self.status = "TIME UP!" # Will be rendered by overlay
                        print("[MANAGER] TIME UP!")
                    
                    # AI Decision (ONLY executed if RUNNING)
                    if ai_pose and human_pose:
                        world_state = {
                            'ai_pose': ai_pose,
                            'human_pose': human_pose,
                            'game_status': self.status
                        }
                        decision = self.ai.decide(world_state)
                        
                        if decision.get('fire_request'):
                            self._attempt_fire("ai", ai_pose, human_pose)
                        
                        # Logging Strategy
                        v_cmd = decision.get('v', 0)
                        w_cmd = decision.get('w', 0)
                        state_ai = decision.get('state', 'UNKNOWN')
                        print(f"[STRATEGY] State: {state_ai} | v={v_cmd:.2f} w={w_cmd:.2f}")

                        # Note: Velocity commands would be sent to ROS bridge here
                        if not self.mock_mode and self.ros_bridge:
                             self.ros_bridge.send_velocity_command(4, v_cmd, w_cmd)
                else:
                    # AI Stopped (No Tick) - Send Stop command once if needed
                    # Ideally we send stop when transitioning to WAITING
                    pass

                # 5. Render
                render_state = {
                    "status": self.status,
                    "scores": self.scores,
                    "entities": {},
                    "events": []
                }
                if ai_pose: render_state["entities"]["ai"] = {"pos": ai_pose}
                if human_pose: render_state["entities"]["human"] = {"pos": human_pose}
                
                self.renderer.update(render_state)
                
        except KeyboardInterrupt:
            print("[MANAGER] Interrupted.")
        except Exception as e:
            print(f"[MANAGER] Crash: {e}")
            import traceback
            traceback.print_exc()
        finally:
            if self.camera: self.camera.stop()
            if self.renderer: self.renderer.close()
            if self.ros_bridge: 
                self.ros_bridge.send_stop_command(4)
                self.ros_bridge.disconnect()

class GameManager:
    """
    Orchestrateur Central du système Tank Arena.
    --------------------------------------------
    Rôle :
    - Fusionne la Perception (Caméra/ArUco) pour mettre à jour le Modèle du Monde.
    - Exécute la Logique de Jeu (Règles, Scores, Cooldowns).
    - Interroge la Stratégie IA pour les décisions.
    - Envoie les commandes de contrôle aux robots physiques via le Pont ROS.
    - Pilote la Visualisation via GameRenderer.
    
    Architecture :
    - Basée sur une boucle : La méthode run() est le cœur du système (cible 60 Hz).
    - Pilotée par état : Utilise 'status' (WAITING, RUNNING) pour contrôler le flux.
    - Configuration : Entièrement pilotée par les fichiers YAML dans /config.
    """
    def __init__(self, mock=False):
        self.root_dir = ROOT_DIR
        self.config_dir = self.root_dir / 'config'
        
        # 1. Load Calibration & Configs
        self.tm = load_calibration(str(self.config_dir))
        self.configs = self._load_configs()
        
        # Game Configs
        game_cfg = self.configs.get('game', {})
        self.game_duration = game_cfg.get('duration', 180)
        
        shot_cfg = game_cfg.get('shooting', {})
        self.SHOT_COOLDOWN = shot_cfg.get('cooldown', 1.0)
        self.MAX_SHOT_DIST = shot_cfg.get('max_range', 10.0)
        self.HIT_RADIUS = shot_cfg.get('hit_radius', 0.25)
        
        # Mock Configs
        mock_cfg = game_cfg.get('mock', {})
        # Note: Config stores relative pos [x%, y%, theta]
        mock_ai = mock_cfg.get('ai_start_pos', [0.2, 0.5, 0.0])
        mock_hu = mock_cfg.get('human_start_pos', [0.8, 0.5, 3.14])
        
        self.mock_ai_pos = [
            self.tm.arena_width_m * mock_ai[0],
            self.tm.arena_height_m * mock_ai[1],
            mock_ai[2]
        ]
        self.mock_hu_pos = [
            self.tm.arena_width_m * mock_hu[0],
            self.tm.arena_height_m * mock_hu[1],
            mock_hu[2]
        ]
        
        # Mock override
        self.mock_mode = mock or (not PERCEPTION_AVAILABLE)
        
        # 2. Setup Components
        self.camera = None
        self.aruco = None
        self._setup_perception()
        
        self.world_model = None
        self._setup_world()
        
        self.ai = None
        self._setup_ai()
        
        self.renderer = None
        self._setup_renderer()
        
        self.ros_bridge = None
        self._setup_control()
        
        # 3. Game State
        self.status = "WAITING" 
        self.scores = {"ai": 0, "human": 0}
        self.start_time = 0
        self.last_shot_time = {"ai": 0, "human": 0}
        self.last_shot_time = {"ai": 0, "human": 0}
        self.last_shot_time = {"ai": 0, "human": 0}
        self.running = True

    def _setup_perception(self):
        if self.mock_mode: return
        print("[MANAGER] Initialisation Perception...")
        try:
            self.camera = RealSenseStream(width=1280, height=720, fps=30)
            self.camera.start()
            self.aruco = ArucoDetector() # Calibration chargée implicitement ou par défaut
        except Exception as e:
            print(f"[MANAGER] Erreur Perception: {e}")
            print("[MANAGER] Passage en MODE MOCK.")
            self.mock_mode = True

    def _setup_world(self):
        print("[MANAGER] Initialisation World Model...")
        self.world_model = WorldModel(
            arena_width_m=self.tm.arena_width_m,
            arena_height_m=self.tm.arena_height_m
        )
        self.world_model.generate_costmap()

    def _setup_ai(self):
        # AI Strategy needs Config
        ia_cfg = self.configs.get('ia', {})
        self.ai = AIStrategy(ia_cfg)
        self.ai.set_world_model(self.world_model)

    def _setup_renderer(self):
        print("[MANAGER] Initialisation Renderer...")
        proj_cfg = self.configs.get('projector', {})
        self.renderer = GameRenderer(
             transform_manager=self.tm,
             config=proj_cfg 
        )

    def _setup_control(self):
        if self.mock_mode: return
        print("[MANAGER] Initialisation ROS Bridge...")
        robot_cfg = self.configs.get('robot', {})
        bridge_cfg = robot_cfg.get('ros_bridge', {})
        
        host = bridge_cfg.get('host', '127.0.0.1')
        port = bridge_cfg.get('port', 8765)
        
        self.ros_bridge = ROSBridgeClient(host, port)
        if self.ros_bridge.connect():
             print("[MANAGER] ROS Bridge Connecté.")
        else:
             print("[MANAGER] ATTENTION: Échec connexion ROS Bridge.")

    def _load_configs(self):
        """Loads all YAML configurations from config/ directory."""
        configs = {}
        for name in ['game', 'robot', 'projector', 'ia']:
            path = self.config_dir / f"{name}.yaml"
            if path.exists():
                with open(path, 'r') as f:
                    configs[name] = yaml.safe_load(f)
            else:
                print(f"[MANAGER] Warning: Config {name}.yaml not found.")
                configs[name] = {}
        return configs

    def _update_perception(self):
        """
        Récupère les frames, détecte les marqueurs ArUco et met à jour les Poses des Robots.
        
        Retourne :
            (ai_pose, human_pose) sous forme [x, y, theta]
        """
        if self.mock_mode:
            return self._mock_perception()
            
        # Real Perception
        color_frame, _ = self.camera.get_frames()
        if color_frame is None: return None, None
        
        detections = self.aruco.detect(color_frame)
        
        ai_pose = None
        human_pose = None
        
        for marker_id, data in detections.items():
            # Extraction données standard
            u, v = data['center']
            th_pix = data['orientation']
            
            # --- Transformation de Coordonnées (Caméra -> Monde) ---
            # Utilisation de UnifiedTransform (Homographie)
            wx, wy = self.tm.camera_to_world(u, v)
            
            # --- Calcul d'Orientation ---
            # Projette un point 20px devant dans l'espace image pour déterminer l'orientation monde.
            # Cette méthode est robuste à la distorsion et correspond à la logique legacy BaseTask.
            u_f = u + 20 * math.cos(th_pix)
            v_f = v + 20 * math.sin(th_pix)
            xf, yf = self.tm.camera_to_world(u_f, v_f)
            
            theta = math.atan2(yf - wy, xf - wx)
            
            # --- MAPPAGE ID (CORRECTIF HISTORIQUE) ---
            # Problème : Dans l'arène physique, les marqueurs étaient inversés par rapport au code.
            #            Le code supposait ID 4 = IA et ID 5 = Humain.
            #            Cependant, le robot Utilisateur avait l'ID 4, causant un tir de l'IA quand l'User appuyait.
            # Fix :      Inversion forcée des IDs ici pour correspondre à la Réalité.
            #            ID 4 -> HUMAIN (Joueur)
            #            ID 5 -> IA (Adversaire)
            
            if marker_id == 4:
                human_pose = [wx, wy, theta]
            elif marker_id == 5:
                ai_pose = [wx, wy, theta]
                    
        return ai_pose, human_pose

    def _mock_perception(self):
        """Simulate robot movements for testing logic."""
        # Simple random walk or static
        # AI
        self.mock_ai_pos[2] += 0.01 # Spin
        
        # Human - Mouse control? Or simple bounce
        # Just static + noise
        noise = (random.random() - 0.5) * 0.02
        self.mock_hu_pos[0] += noise
        
        # Clamp to arena
        self.mock_hu_pos[0] = max(0, min(self.arena_w, self.mock_hu_pos[0]))
        
        return self.mock_ai_pos, self.mock_hu_pos

    def _handle_input(self, human_pose):
        keys = pygame.key.get_pressed()
        
        # Debounce logic
        current_time = time.time()
        if hasattr(self, 'last_key_time') and (current_time - self.last_key_time < 0.2):
            return

        if keys[pygame.K_SPACE]:
            if self.status == "RUNNING":
                print("[MANAGER] STOP (AI Halted)")
                self.status = "WAITING"
            elif self.status == "WAITING" or self.status == "GAME OVER":
                self.status = "RUNNING"
                self.start_time = time.time()
                # self.scores = {"ai": 0, "human": 0} # Optional: reset scores on restart
                print("[MANAGER] START (AI Active)")
            self.last_key_time = current_time
        
        # User Fire (Always possible or only when running? User said "F l'user a tire")
        # Let's assume User can fire anytime for fun, or restrict to RUNNING. User said "pour l'instant on va se contenter quand on le lance que l'user tire" implies RUNNING.
        if keys[pygame.K_f] and self.status == "RUNNING":
             if human_pose:
                 # Target for Human is AI
                 ai_pos_target = self.world_model.get_robot_pose(4)
                 self._attempt_fire("human", human_pose, ai_pos_target)
                 self.last_key_time = current_time

    def _attempt_fire(self, shooter_name, shooter_pose, target_pose):
        now = time.time()
        if now - self.last_shot_time.get(shooter_name, 0) < self.SHOT_COOLDOWN:
            return

        self.last_shot_time[shooter_name] = now
        
        # 1. Physics: Raycasting
        sx, sy, sth = shooter_pose
        cos_th = math.cos(sth)
        sin_th = math.sin(sth)
        
        # Default: Max Range
        dist_final = self.MAX_SHOT_DIST
        
        # Check Wall Intersection (Box Arena)
        candidates = []
        if abs(cos_th) > 0.001:
            t1 = (0 - sx) / cos_th; t2 = (self.world_model.arena_width - sx) / cos_th
            if t1 > 0: candidates.append(t1)
            if t2 > 0: candidates.append(t2)
        if abs(sin_th) > 0.001:
            t3 = (0 - sy) / sin_th; t4 = (self.world_model.arena_height - sy) / sin_th
            if t3 > 0: candidates.append(t3)
            if t4 > 0: candidates.append(t4)
            
        if candidates:
            dist_wall = min(candidates)
            dist_final = min(dist_wall, dist_final)
        
        end_x = sx + dist_final * cos_th
        end_y = sy + dist_final * sin_th
        
        # 2. Check Hit on Target (Circle Intersection)
        hit_confirmed = False
        HIT_RADIUS = 0.20 # Updated to match yaml
        
        if target_pose:
            tx, ty, _ = target_pose
            # Vector Shooter->Target
            idx = tx - sx
            idy = ty - sy
            # Projection scalar
            proj = (idx * cos_th + idy * sin_th)
            
            # If target is in front and within range
            if 0 < proj < dist_final:
                # Closest point on line
                cpx = sx + proj * cos_th
                cpy = sy + proj * sin_th
                
                dist_sq = (cpx - tx)**2 + (cpy - ty)**2
                if dist_sq < (HIT_RADIUS**2):
                    hit_confirmed = True
                    # Snap shot end to impact point roughly
                    end_x, end_y = cpx, cpy

        # 3. Resolve
        is_ai = (shooter_name == "ai")
        if hit_confirmed:
            self.scores[shooter_name] += 1
            self.renderer.trigger_hit([end_x, end_y])
            print(f"[MANAGER] HIT! {shooter_name} scored.")
            
        self.renderer.trigger_shot([sx, sy], [end_x, end_y], is_ai)

    def run(self):
        print("[MANAGER] Démarrage de la boucle principale (60Hz)...")
        clock = pygame.time.Clock()
        
        try:
            while self.running:
                dt = clock.tick(60) / 1000.0
                
                # 1. Perception
                ai_pose, human_pose = self._update_perception()
                
                # DEBUG: Print perception status periodically
                if int(time.time() * 10) % 20 == 0: 
                     print(f"[DEBUG] Pose AI: {ai_pose}, Human: {human_pose}")

                # 2. Input
                self._handle_input(human_pose)
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False
                
                # 3. Logic & AI
                if self.status == "RUNNING":
                    # Time Check
                    elapsed = time.time() - self.start_time
                    if elapsed > self.game_duration:
                        self.status = "GAME OVER"
                        print("[MANAGER] GAME OVER - Temps écoulé")
                    
                    # AI Decision
                    if ai_pose:
                        # Update World Model
                        if human_pose:
                             self.world_model.update_robot_pose(4, human_pose)
                        self.world_model.update_robot_pose(5, ai_pose)
                        
                        # Strategy Decision
                        state = {
                            'ai_pose': ai_pose,
                            'human_pose': human_pose,
                            'game_status': self.status
                        }
                        cmd = self.ai.decide(state)
                        
                        # Send Control
                        v = cmd.get('v', 0.0)
                        w = cmd.get('w', 0.0)
                        fire_req = cmd.get('fire_request', False)
                        
                        if int(time.time() * 10) % 20 == 0:
                            print(f"[DEBUG] CMD -> V={v:.2f}, W={w:.2f}, FIRE={fire_req}")

                        if self.ros_bridge:
                            self.ros_bridge.send_velocity_command(5, v, w)
                            
                        # Handle AI Fire
                        if fire_req and human_pose:
                            self._attempt_fire("ai", ai_pose, human_pose)

                else:
                    # Safety Stop
                    if self.ros_bridge:
                        self.ros_bridge.send_velocity_command(5, 0.0, 0.0)
                
                # 4. Render
                entities = {}
                if ai_pose:
                    entities['ai'] = {'pos': ai_pose}
                if human_pose:
                    entities['human'] = {'pos': human_pose}
                    
                state_dataset = {
                    'status': self.status,
                    'scores': self.scores,
                    'time_left': max(0, self.game_duration - (time.time() - self.start_time)) if self.status == "RUNNING" else 0,
                    'entities': entities
                }
                self.renderer.render(state_dataset)
                
        except KeyboardInterrupt:
            print("\n[MANAGER] Arrêt manuel (Ctrl+C)")
        finally:
            self.close()

    def close(self):
        print("[MANAGER] Fermeture des composants...")
        if self.camera: self.camera.stop()
        if self.ros_bridge: 
            self.ros_bridge.send_stop_command(5)
            self.ros_bridge.disconnect()
        pygame.quit()
        print("[MANAGER] Bye!")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--mock', action='store_true', help="Run in Mock Mode (no hardware)")
    args = parser.parse_args()
    
    gm = GameManager(mock=args.mock)
    gm.run()
