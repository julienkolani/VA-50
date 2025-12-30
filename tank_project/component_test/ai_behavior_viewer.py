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
from core.world.unified_transform import load_calibration

# --- CONFIGURATION VISUELLE ---
C_BG = (5, 5, 15)
C_RETRAIT = (255, 50, 50)   # Rouge
C_ATTAQUE = (0, 255, 100)   # Vert
C_POURSUITE = (0, 150, 255) # Bleu
C_CYAN = (0, 255, 255)      # IA
C_WHITE = (255, 255, 255)

# --- PARAMÈTRES IA ---
ROBOT_AI_ID = 5
ROBOT_ENEMY_ID = 4
SEUIL_MENACE_DEG = 15.0     # Angle max pour considérer que l'ennemi nous vise
SEUIL_LOCK_TIR_DEG = 5.0    # Précision requise pour l'état de tir

class TankBehaviorTree:
    """Arbre de comportement tactique en Français."""
    def __init__(self, world_model):
        self.world = world_model
        self.etat = "INITIALISATION"

    def check_ligne_de_vue(self, p1, p2):
        """Vérifie si le trajet est libre d'obstacles."""
        steps = 20
        for i in range(1, steps):
            tx = p1[0] + (p2[0] - p1[0]) * (i / steps)
            ty = p1[1] + (p2[1] - p1[1]) * (i / steps)
            # Utilise la grille d'occupation pour valider la position
            if not self.world.is_position_valid(tx, ty):
                return False
        return True

    def est_vise_par_ennemi(self, ai_pose, en_pose):
        """Détermine si l'ennemi pointe son canon vers l'IA avec une ligne de vue claire."""
        # Vecteur Ennemi -> IA
        dx, dy = ai_pose[0] - en_pose[0], ai_pose[1] - en_pose[1]
        angle_vers_ia = np.arctan2(dy, dx)
        
        # Erreur entre l'orientation de l'ennemi et la position de l'IA
        erreur_angle = (en_pose[2] - angle_vers_ia + np.pi) % (2 * np.pi) - np.pi
        erreur_deg = abs(np.degrees(erreur_angle))
        
        # Condition : L'ennemi regarde vers nous ET pas d'obstacle entre nous
        if erreur_deg < SEUIL_MENACE_DEG:
            return self.check_ligne_de_vue(en_pose[:2], ai_pose[:2])
        return False

    def update(self, ai_pose, en_pose):
        """Décide de l'état selon la hiérarchie des priorités."""
        if ai_pose is None or en_pose is None:
            self.etat = "RECHERCHE DE CIBLES"
            return self.etat

        # 1. PRIORITÉ : SURVIE (Seulement si visé directement)
        if self.est_vise_par_ennemi(ai_pose, en_pose):
            if self.etat != "RETRAIT (MENACE DIRECTE)":
                print(f"[BT] ÉTAT : RETRAIT - L'ennemi a une ligne de tir claire !")
            self.etat = "RETRAIT (MENACE DIRECTE)"
            return self.etat

        # 2. PRIORITÉ : ATTAQUE (Si ligne de vue sur l'ennemi)
        if self.check_ligne_de_vue(ai_pose[:2], en_pose[:2]):
            dx, dy = en_pose[0] - ai_pose[0], en_pose[1] - ai_pose[1]
            angle_vers_en = np.arctan2(dy, dx)
            erreur_ia = (ai_pose[2] - angle_vers_en + np.pi) % (2 * np.pi) - np.pi
            
            if abs(np.degrees(erreur_ia)) < SEUIL_LOCK_TIR_DEG:
                self.etat = "ATTAQUE (VERROUILLÉ)"
            else:
                self.etat = "ATTAQUE (ALIGNEMENT)"
            return self.etat

        # 3. PRIORITÉ : POURSUITE (Par défaut)
        if self.etat != "POURSUITE":
            print(f"[BT] ÉTAT : POURSUITE - Ennemi hors de vue ou caché.")
        self.etat = "POURSUITE"
        return self.etat

def get_robot_pose_in_world(detection_data, transform_mgr):
    u, v = detection_data['center']
    theta_pix = detection_data['orientation']
    u_f = u + 20 * np.cos(theta_pix)
    v_f = v + 20 * np.sin(theta_pix)
    x, y = transform_mgr.camera_to_world(u, v)
    xf, yf = transform_mgr.camera_to_world(u_f, v_f)
    # Protection contre les sorties de grille
    x = np.clip(x, 0.01, transform_mgr.arena_width_m - 0.01)
    y = np.clip(y, 0.01, transform_mgr.arena_height_m - 0.01)
    return x, y, np.arctan2(yf - y, xf - x)

def main():
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
    font_etat = pygame.font.SysFont("Impact", 50)
    font_debug = pygame.font.SysFont("Consolas", 18)

    camera = RealSenseStream(width=1280, height=720, fps=30)
    camera.start()
    aruco = ArucoDetector()
    
    # WorldModel pour l'accès aux obstacles
    world = WorldModel(transform_mgr.arena_width_m, transform_mgr.arena_height_m)
    world.generate_costmap() #
    
    brain = TankBehaviorTree(world)

    try:
        while True:
            screen.fill(C_BG)
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE: return

            color_frame, _ = camera.get_frames()
            if color_frame is None: continue
            detections = aruco.detect(color_frame)

            ai_pose = en_pose = None
            if ROBOT_AI_ID in detections:
                ai_pose = get_robot_pose_in_world(detections[ROBOT_AI_ID], transform_mgr)
            if ROBOT_ENEMY_ID in detections:
                en_pose = get_robot_pose_in_world(detections[ROBOT_ENEMY_ID], transform_mgr)

            # Mise à jour de l'intelligence
            mode = brain.update(ai_pose, en_pose)

            # --- RENDU VISUEL ---
            if ai_pose and en_pose:
                ai_px = transform_mgr.world_to_projector(ai_pose[0], ai_pose[1])
                en_px = transform_mgr.world_to_projector(en_pose[0], en_pose[1])
                
                # Couleur dynamique
                color = C_POURSUITE
                if "RETRAIT" in mode: color = C_RETRAIT
                if "ATTAQUE" in mode: color = C_ATTAQUE

                # Dessin du laser de visée
                pygame.draw.line(screen, color, ai_px, en_px, 2)
                
                # Robots
                pygame.draw.circle(screen, C_CYAN, ai_px, 15, 2)
                pygame.draw.circle(screen, C_RETRAIT, en_px, 15, 2)

            # Affichage de l'état central
            txt_surf = font_etat.render(f"MODE IA : {mode}", True, C_WHITE)
            screen.blit(txt_surf, (screen.get_width()//2 - txt_surf.get_width()//2, 50))

            # Debug logs en bas de l'écran
            if ai_pose and en_pose:
                dist = np.hypot(ai_pose[0]-en_pose[0], ai_pose[1]-en_pose[1])
                debug_txt = [
                    f"IA Pose: x={ai_pose[0]:.2f} y={ai_pose[1]:.2f} th={np.degrees(ai_pose[2]):.0f}°",
                    f"EN Pose: x={en_pose[0]:.2f} y={en_pose[1]:.2f} th={np.degrees(en_pose[2]):.0f}°",
                    f"Distance: {dist:.2f}m | Menace: {'OUI' if brain.est_vise_par_ennemi(ai_pose, en_pose) else 'NON'}"
                ]
                y_y = 650
                for line in debug_txt:
                    s = font_debug.render(line, True, C_WHITE)
                    screen.blit(s, (30, y_y))
                    y_y += 25

            pygame.display.flip()
            pygame.time.Clock().tick(30)

    except KeyboardInterrupt: pass
    finally:
        camera.stop()
        pygame.quit()

if __name__ == '__main__': main()