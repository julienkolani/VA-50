import sys
import os
import pygame
import numpy as np
import time
import yaml
import cv2
from pathlib import Path
from abc import ABC, abstractmethod

# Imports système de base nécessaires au standalone
from perception.camera.realsense_stream import RealSenseStream
from perception.camera.aruco_detector import ArucoDetector
from core.world.world_model import WorldModel
from core.world.unified_transform import load_calibration
from core.control.ros_bridge_client import ROSBridgeClient

class BaseTask(ABC):
    """
    Interface de base pour toutes les tâches du robot.
    Permet l'exécution intégrée (via Strategy) ou standalone (via __main__).
    """
    
    def __init__(self, name: str, config: dict = None):
        self.name = name
        self.config = config or {}
        
    @abstractmethod
    def execute(self, context: dict) -> dict:
        """
        Exécute la logique de la tâche pour un tick.
        
        Args:
            context: Dictionnaire contenant l'état du monde
                - 'ai_pose': (x, y, theta)
                - 'human_pose': (x, y, theta)
                - 'world': WorldModel instance
                - 'dt': delta time (s)
                
        Returns:
            dict: Commandes pour le robot
                - 'v': Vitesse linéaire (m/s)
                - 'w': Vitesse angulaire (rad/s)
                - 'fire_request': bool
                - 'status': str (RUNNING, SUCCESS, FAILURE)
                - 'debug_info': dict (pour affichage)
        """
        pass

    def draw(self, surface, transform_mgr, debug_info):
        """
        Méthode optionnelle pour dessiner des infos de debug sur l'écran Pygame.
        """
        pass

    @classmethod
    def run_standalone(cls):
        """
        Lance la tâche en mode autonome avec toute l'infrastructure (Caméra, Pygame, ROS).
        Copie intelligemment la logique de test de component_test.
        """
        print(f"[TASK] Lancement Standalone : {cls.__name__}")
        
        # 1. Config & Calibration
        root_dir = Path(__file__).parent.parent.parent
        config_dir = root_dir / 'config'
        transform_mgr = load_calibration(str(config_dir))
        
        proj_conf = {}
        if (config_dir / 'projector.yaml').exists():
            with open(config_dir / 'projector.yaml') as f:
                proj_conf = yaml.safe_load(f)
        
        # 2. Setup Pygame
        os.environ['SDL_VIDEO_WINDOW_POS'] = "{},{}".format(
            proj_conf.get('display', {}).get('monitor_offset_x', 1920),
            0
        )
        pygame.init()
        screen = pygame.display.set_mode((1024, 768), pygame.NOFRAME)
        clock = pygame.time.Clock()
        font = pygame.font.SysFont("Consolas", 18)

        # 3. Hardware
        camera = RealSenseStream(width=1280, height=720, fps=30)
        camera.start()
        aruco = ArucoDetector()
        
        # 4. World Model
        world = WorldModel(transform_mgr.arena_width_m, transform_mgr.arena_height_m)
        world.generate_costmap()
        
        # 5. Robot Connection
        ros_bridge = ROSBridgeClient(host='127.0.0.1', port=8765)
        connected = ros_bridge.connect()
        if not connected:
            print("[WARN] Robot non connecté (Mode Simulation)")

        # 6. Instanciation de la Tâche
        task = cls() # Suppose constructeur par défaut
        
        print("\n[INFO] Tâche démarrée. ESC pour quitter.\n")
        
        running = True
        try:
            while running:
                dt = clock.tick(30) / 1000.0
                screen.fill((10, 10, 30))
                
                # Input
                for event in pygame.event.get():
                    if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                        running = False
                
                # Perception
                color_frame, _ = camera.get_frames()
                if color_frame is None: continue
                detections = aruco.detect(color_frame)
                
                ai_pose = None
                human_pose = None
                
                # Note: IDs codés en dur pour le standalone (4=AI, 5=Enemy)
                if 5 in detections:
                    ai_pose = cls._get_pose(detections[5], transform_mgr)
                if 4 in detections:
                    human_pose = cls._get_pose(detections[4], transform_mgr)
                
                # Context Build
                context = {
                    'ai_pose': ai_pose, # Standalone inverse souvent les IDS, à vérifier
                    'human_pose': human_pose,
                    'world': world,
                    'dt': dt
                }
                
                # --- EXECUTION TÂCHE ---
                # Dans component_test, AI=5 et Enemy=4. 
                # Adaptons le mapping pour que le standalone matche le jeu
                # Jeu : AI=4, Enemy=5.
                # Component Test : AI=5, Enemy=4.
                # On va dire : Robot contrôlé = celui défini dans la tache
                
                cmd = task.execute(context)
                
                # --- COMMANDE ROBOT ---
                if connected and ai_pose: # Si on a détecté notre robot
                     # Attention aux IDs ! Dans component_test AI=5.
                     # On envoie la commande au robot 5
                     ros_bridge.send_velocity_command(5, cmd.get('v', 0), cmd.get('w', 0))
                
                # --- RENDU ---
                # Dessin basique robots
                if ai_pose:
                    px = transform_mgr.world_to_projector(ai_pose[0], ai_pose[1])
                    pygame.draw.circle(screen, (0, 255, 255), px, 15, 2)
                if human_pose:
                    px = transform_mgr.world_to_projector(human_pose[0], human_pose[1])
                    pygame.draw.circle(screen, (255, 50, 50), px, 15, 2)
                    
                # Dessin Tâche
                task.draw(screen, transform_mgr, cmd.get('debug_info', {}))
                
                # Texte Status
                txt = f"Status: {cmd.get('status', 'N/A')} | v={cmd.get('v',0):.2f} w={cmd.get('w',0):.2f}"
                screen.blit(font.render(txt, True, (255, 255, 255)), (10, 10))
                
                pygame.display.flip()
                
        except KeyboardInterrupt:
            pass
        finally:
            if connected: ros_bridge.send_velocity_command(5, 0, 0)
            camera.stop()
            pygame.quit()

    @staticmethod
    def _get_pose(data, tm):
        # Utilitaire rapide pour standalone
        u, v = data['center']
        th_pix = data['orientation']
        x, y = tm.camera_to_world(u, v)
        u_f = u + 20 * np.cos(th_pix)
        v_f = v + 20 * np.sin(th_pix)
        xf, yf = tm.camera_to_world(u_f, v_f)
        th = np.arctan2(yf-y, xf-x)
        return (x, y, th)
