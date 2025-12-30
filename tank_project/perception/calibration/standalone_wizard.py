#!/usr/bin/env python3
"""
CALIBRATION WIZARD STANDALONE - Intégré au Projet
--------------------------------------------------
Wizard de calibration basé sur l'homographie directe Caméra → Projecteur.
Sauvegarde les données en JSON pour une meilleure fiabilité.

Usage:
    python -m perception.calibration.standalone_wizard

Auteur: Julien (version standalone validée)
"""

import sys
import os
import time
import json
import yaml
import cv2
import numpy as np
import pygame
import pyrealsense2 as rs
from datetime import datetime
from pathlib import Path


class StandaloneCamera:
    """Gestionnaire Caméra RealSense avec correction de distorsion."""
    
    def __init__(self, width=1280, height=720, fps=30):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.profile = self.pipeline.start(config)
        
        # Warmup
        time.sleep(1.0)
        self.K, self.D = self._get_intrinsics()
        
    def _get_intrinsics(self):
        """Récupère les paramètres intrinsèques de la caméra."""
        stream = self.profile.get_stream(rs.stream.color)
        intr = stream.as_video_stream_profile().get_intrinsics()
        K = np.array([
            [intr.fx, 0, intr.ppx],
            [0, intr.fy, intr.ppy],
            [0, 0, 1]
        ], dtype=np.float32)
        D = np.array(intr.coeffs, dtype=np.float32)
        print(f"[CAM] Intrinsèques: fx={intr.fx:.1f}, fy={intr.fy:.1f}, cx={intr.ppx:.1f}, cy={intr.ppy:.1f}")
        return K, D

    def get_frame(self):
        """Récupère une frame corrigée de la distorsion."""
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        img = np.asanyarray(color_frame.get_data())
        # Correction Distorsion Immédiate
        return cv2.undistort(img, self.K, self.D)

    def stop(self):
        """Arrête le pipeline caméra."""
        self.pipeline.stop()


class CalibrationData:
    """
    Structure de données de calibration.
    
    Stockage hybride :
    - calibration.npy : Matrices numpy (H_CamToProj, K, D)
    - calibration_meta.json : Métadonnées (date, échelle, version)
    - projector.yaml : Config manuelle (résolution, marges) - existant
    """
    
    def __init__(self):
        self.H_CamToProj = None      # Homographie Caméra → Projecteur (3x3)
        self.pixels_per_meter = 0.0   # Échelle en pixels/mètre
        self.proj_width = 0
        self.proj_height = 0
        self.margin = 0
        self.marker_size_m = 0.0
        self.calibration_date = None
        
        # Intrinsèques caméra (pour référence)
        self.camera_K = None
        self.camera_D = None
        
    def save(self, config_dir: str):
        """
        Sauvegarde la calibration en format hybride.
        
        Args:
            config_dir: Dossier de configuration (ex: tank_project/config/)
            
        Crée:
            - calibration.npy : Matrices numpy
            - calibration_meta.json : Métadonnées
        """
        config_path = Path(config_dir)
        
        # 1. Matrices numpy → .npz (np.savez génère un fichier .npz)
        npz_path = config_path / "calibration.npz"
        np.savez(
            npz_path,
            H_CamToProj=self.H_CamToProj,
            camera_K=self.camera_K,
            camera_D=self.camera_D
        )
        print(f"[CALIB] Matrices sauvées: {npz_path}")
        
        # 2. Métadonnées → .json
        meta_path = config_path / "calibration_meta.json"
        meta = {
            'version': '2.1',
            'calibration_date': self.calibration_date,
            'scale': {
                'pixels_per_meter': self.pixels_per_meter,
                'marker_size_m': self.marker_size_m
            },
            'projector': {
                'width': self.proj_width,
                'height': self.proj_height,
                'margin': self.margin
            }
        }
        with open(meta_path, 'w') as f:
            json.dump(meta, f, indent=2)
        print(f"[CALIB] Métadonnées sauvées: {meta_path}")
        
    @classmethod
    def load(cls, config_dir: str) -> 'CalibrationData':
        """
        Charge la calibration depuis les fichiers hybrides.
        
        Args:
            config_dir: Dossier de configuration
            
        Returns:
            CalibrationData initialisé
        """
        config_path = Path(config_dir)
        calib = cls()
        
        # 1. Charger matrices numpy (.npz)
        npz_path = config_path / "calibration.npz"
        if npz_path.exists():
            data = np.load(npz_path)
            if 'H_CamToProj' in data:
                calib.H_CamToProj = data['H_CamToProj']
            if 'camera_K' in data:
                calib.camera_K = data['camera_K']
            if 'camera_D' in data:
                calib.camera_D = data['camera_D']
            print(f"[CALIB] Matrices chargées: {npz_path}")
        
        # 2. Charger métadonnées JSON
        meta_path = config_path / "calibration_meta.json"
        if meta_path.exists():
            with open(meta_path, 'r') as f:
                meta = json.load(f)
            
            calib.calibration_date = meta.get('calibration_date')
            calib.pixels_per_meter = meta['scale']['pixels_per_meter']
            calib.marker_size_m = meta['scale']['marker_size_m']
            calib.proj_width = meta['projector']['width']
            calib.proj_height = meta['projector']['height']
            calib.margin = meta['projector']['margin']
            print(f"[CALIB] Métadonnées chargées: {meta_path}")
            
        return calib


class StandaloneCalibrationWizard:
    """
    Wizard de calibration standalone.
    Calcule une homographie directe Caméra → Projecteur.
    """
    
    # Configuration par défaut (peut être overridé)
    DEFAULT_CONFIG = {
        'proj_width': 1024,
        'proj_height': 768,
        'offset_x': 1920,
        'offset_y': 0,
        'margin': 50,
        'marker_size_m': 0.10,
        'marker_size_px': 150
    }
    
    def __init__(self, config: dict = None):
        """
        Initialise le wizard.
        
        Args:
            config: Dictionnaire de configuration optionnel
        """
        cfg = {**self.DEFAULT_CONFIG, **(config or {})}
        
        self.proj_w = cfg['proj_width']
        self.proj_h = cfg['proj_height']
        self.offset_x = cfg['offset_x']
        self.offset_y = cfg['offset_y']
        self.margin = cfg['margin']
        self.marker_size_m = cfg['marker_size_m']
        self.marker_size_px = cfg['marker_size_px']
        
        # Hardware
        self.cam = None
        self.screen = None
        
        # ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Store full config
        self.config = cfg
        
        # Fonts
        self.font = None
        self.small_font = None
        
        # State
        self.step = 0
        self.calibration = CalibrationData()
        self.running = True
        
    def _init_hardware(self):
        """Initialise la caméra et le projecteur."""
        print("[WIZARD] Initialisation Caméra RealSense...")
        # Utiliser la config si disponible, sinon défauts
        w = self.config.get('camera_width', 1280)
        h = self.config.get('camera_height', 720)
        fps = self.config.get('camera_fps', 30)
        
        self.cam = StandaloneCamera(width=w, height=h, fps=fps)
        self.calibration.camera_K = self.cam.K
        self.calibration.camera_D = self.cam.D
        
        print(f"[WIZARD] Initialisation Projecteur ({self.proj_w}x{self.proj_h} à {self.offset_x},{self.offset_y})...")
        os.environ['SDL_VIDEO_WINDOW_POS'] = f"{self.offset_x},{self.offset_y}"
        pygame.init()
        self.screen = pygame.display.set_mode(
            (self.proj_w, self.proj_h), 
            pygame.NOFRAME | pygame.DOUBLEBUF
        )
        pygame.mouse.set_visible(False)
        pygame.display.set_caption("Calibration Wizard")
        
        self.font = pygame.font.SysFont("Arial", 40, bold=True)
        self.small_font = pygame.font.SysFont("Arial", 24)
        
        # Store config in calibration data
        self.calibration.proj_width = self.proj_w
        self.calibration.proj_height = self.proj_h
        self.calibration.margin = self.margin
        self.calibration.marker_size_m = self.marker_size_m

    def _draw_text_center(self, text, y_off=0, color=(0, 0, 0), bg=(255, 255, 255)):
        """Dessine du texte centré."""
        surf = self.font.render(text, True, color, bg)
        rect = surf.get_rect(center=(self.proj_w // 2, self.proj_h // 2 + y_off))
        self.screen.blit(surf, rect)

    def _get_corner_positions(self):
        """
        Retourne les positions des 4 coins (centres des marqueurs).
        ORDRE CRITIQUE : Cohérent avec les IDs ArUco.
        """
        s = self.marker_size_px
        half = s // 2
        m = self.margin
        
        # IDs 0-3 dans l'ordre : Haut-Gauche, Haut-Droite, Bas-Droite, Bas-Gauche
        return {
            0: (m + half, m + half),                          # Top-Left
            1: (self.proj_w - m - half, m + half),            # Top-Right
            2: (self.proj_w - m - half, self.proj_h - m - half),  # Bottom-Right
            3: (m + half, self.proj_h - m - half)             # Bottom-Left
        }

    def _draw_markers(self, positions: dict, size: int):
        """Dessine les marqueurs ArUco aux positions spécifiées."""
        for marker_id, pos in positions.items():
            # Génère l'image du marqueur
            img = cv2.aruco.generateImageMarker(self.aruco_dict, marker_id, size)
            img_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            img_rgb = np.transpose(img_rgb, (1, 0, 2))
            surf = pygame.surfarray.make_surface(img_rgb)
            rect = surf.get_rect(center=pos)
            self.screen.blit(surf, rect)
            
            # Affiche l'ID
            lbl = self.small_font.render(str(marker_id), True, (0, 0, 0))
            lbl_rect = lbl.get_rect(center=pos)
            pygame.draw.rect(self.screen, (255, 255, 255), lbl_rect.inflate(4, 4))
            self.screen.blit(lbl, lbl_rect)

    def _handle_input(self) -> str:
        """Gère les entrées utilisateur. Retourne 'space', 'escape', ou None."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return 'escape'
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return 'escape'
                if event.key == pygame.K_SPACE:
                    return 'space'
        return None

    def run(self, output_path: str = None) -> CalibrationData:
        """
        Exécute le wizard de calibration.
        
        Args:
            output_path: Chemin de sauvegarde du fichier JSON (optionnel)
            
        Returns:
            CalibrationData: Données de calibration
        """
        self._init_hardware()
        
        try:
            while self.running:
                action = self._handle_input()
                if action == 'escape':
                    self.running = False
                    break
                
                self.screen.fill((255, 255, 255))
                
                if self.step == 0:
                    self._step_intro(action)
                elif self.step == 1:
                    self._step_geometric(action)
                elif self.step == 2:
                    self._step_metric(action)
                elif self.step == 3:
                    self._step_verify()
                    
                pygame.display.flip()
                
        finally:
            if self.cam:
                self.cam.stop()
            pygame.quit()
        
        # Sauvegarde si chemin fourni
        if output_path and self.calibration.H_CamToProj is not None:
            self.calibration.calibration_date = datetime.now().isoformat()
            self.calibration.save(output_path)
            
        return self.calibration

    def _step_intro(self, action):
        """Étape 0: Introduction."""
        self.screen.fill((0, 0, 0))
        self._draw_text_center("CALIBRATION WIZARD", -80, (0, 255, 0), (0, 0, 0))
        self._draw_text_center("1. Videz l'arène", 0, (255, 255, 255), (0, 0, 0))
        self._draw_text_center("2. Vérifiez que la caméra voit tout", 50, (255, 255, 255), (0, 0, 0))
        self._draw_text_center("ESPACE pour commencer", 120, (255, 255, 0), (0, 0, 0))
        
        if action == 'space':
            self.step = 1

    def _step_geometric(self, action):
        """Étape 1: Calibration géométrique (4 coins)."""
        corners = self._get_corner_positions()
        self._draw_markers(corners, self.marker_size_px)
        self._draw_text_center("La caméra voit-elle les 4 coins ?", 0)
        self._draw_text_center("ESPACE pour capturer", 50)
        
        if action == 'space':
            self._capture_geometric(corners)

    def _capture_geometric(self, expected_corners: dict):
        """Capture et calcule l'homographie géométrique."""
        print("[WIZARD] Capture géométrique...")
        img = self.cam.get_frame()
        corners, ids, _ = self.detector.detectMarkers(img)
        
        if ids is None or len(ids) < 4:
            found = len(ids) if ids is not None else 0
            print(f"[WIZARD] ERREUR: Seulement {found}/4 marqueurs détectés")
            if ids is not None:
                print(f"[WIZARD] IDs vus: {ids.flatten().tolist()}")
            return
        
        # Collecte les correspondances
        src_list = []  # Points caméra
        dst_list = []  # Points projecteur
        
        flat_ids = ids.flatten()
        for i, marker_id in enumerate(flat_ids):
            if marker_id in expected_corners:
                center_cam = corners[i][0].mean(axis=0)
                center_proj = expected_corners[marker_id]
                src_list.append(center_cam)
                dst_list.append(center_proj)
        
        if len(src_list) < 4:
            print("[WIZARD] ERREUR: IDs manquants (besoin 0,1,2,3)")
            return
        
        # Calcul homographie directe Caméra → Projecteur
        self.calibration.H_CamToProj, _ = cv2.findHomography(
            np.array(src_list), 
            np.array(dst_list)
        )
        print("[WIZARD] Homographie Caméra->Projecteur calculée!")
        self.step = 2

    def _step_metric(self, action):
        """Étape 2: Calibration métrique (échelle)."""
        self._draw_text_center("Posez le robot (ID 4 ou 5) au centre", -50)
        self._draw_text_center(f"Taille marqueur: {self.marker_size_m*100:.0f} cm", 0)
        self._draw_text_center("ESPACE pour mesurer l'échelle", 50)
        
        if action == 'space':
            self._capture_metric()

    def _capture_metric(self):
        """Calcule l'échelle pixels/mètre (Robuste: Médiane sur 60 frames)."""
        print("[WIZARD] Capture métrique... (Patientez ~2s)")
        
        scales = []
        target_frames = 60
        
        for _ in range(target_frames):
            img = self.cam.get_frame()
            corners, ids, _ = self.detector.detectMarkers(img)
            
            robot_idx = -1
            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id in [4, 5]:
                        robot_idx = i
                        break
            
            if robot_idx != -1:
                # Transforme les coins du marqueur vers l'espace projecteur
                c_cam = corners[robot_idx][0]
                c_proj = cv2.perspectiveTransform(
                    np.array([c_cam]), 
                    self.calibration.H_CamToProj
                )[0]
                
                # Mesure la taille en pixels projecteur
                w1 = np.linalg.norm(c_proj[0] - c_proj[1])
                h1 = np.linalg.norm(c_proj[1] - c_proj[2])
                avg_px = (w1 + h1) / 2.0
                
                current_scale = avg_px / self.marker_size_m
                scales.append(current_scale)
            
            # Petit délai pour laisser varier le bruit
            time.sleep(0.01)
            
        if not scales:
             print("[WIZARD] ERREUR: Aucun robot (ID 4/5) détecté sur la durée")
             return

        # Calcul robuste : Médiane
        self.calibration.pixels_per_meter = float(np.median(scales))
        print(f"[WIZARD] Échelle (Médiane/{len(scales)}): {self.calibration.pixels_per_meter:.2f} px/m")
        self.step = 3

    def _step_verify(self):
        """Étape 3: Vérification AR temps réel."""
        self.screen.fill((0, 0, 0))
        
        img = self.cam.get_frame()
        if img is None:
            return
        
        # Warp l'image caméra vers l'espace projecteur
        warped = cv2.warpPerspective(
            img, 
            self.calibration.H_CamToProj, 
            (self.proj_w, self.proj_h)
        )
        
        # Conversion pour Pygame
        warped = cv2.cvtColor(warped, cv2.COLOR_BGR2RGB)
        warped = np.transpose(warped, (1, 0, 2))
        surf = pygame.surfarray.make_surface(warped)
        
        # Grille de référence
        self._draw_grid()
        
        # Fantôme semi-transparent
        surf.set_alpha(150)
        self.screen.blit(surf, (0, 0))
        
        # Tracking robot en temps réel
        corners, ids, _ = self.detector.detectMarkers(img)
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id in [4, 5]:
                    c_cam = corners[i][0].mean(axis=0)
                    c_proj = cv2.perspectiveTransform(
                        np.array([[c_cam]]), 
                        self.calibration.H_CamToProj
                    )[0][0]
                    px, py = int(c_proj[0]), int(c_proj[1])
                    
                    # Point cyan pour le robot
                    pygame.draw.circle(self.screen, (0, 255, 255), (px, py), 10)
                    pygame.draw.circle(self.screen, (0, 255, 255), (px, py), 20, 2)
        
        # UI
        label = self.small_font.render(
            f"AR CHECK - Échelle: {self.calibration.pixels_per_meter:.1f} px/m | ESC=Quitter", 
            True, (0, 255, 0)
        )
        self.screen.blit(label, (10, 10))
        self._draw_text_center(
            "Le point cyan doit être sur le robot réel", 
            300, (255, 255, 0), (0, 0, 0)
        )

    def _draw_grid(self):
        """Dessine une grille de référence (50cm)."""
        if self.calibration.pixels_per_meter == 0:
            return
            
        px_step = int(0.5 * self.calibration.pixels_per_meter)
        if px_step <= 0:
            px_step = 100
        
        col = (0, 100, 0)
        for x in range(0, self.proj_w, px_step):
            pygame.draw.line(self.screen, col, (x, 0), (x, self.proj_h))
        for y in range(0, self.proj_h, px_step):
            pygame.draw.line(self.screen, col, (0, y), (self.proj_w, y))


def main():
    """Point d'entrée principal."""
    # Dossier de configuration
    project_root = Path(__file__).parent.parent.parent
    config_dir = project_root / "config"
    
    print(f"[WIZARD] Dossier config: {config_dir}")
    
    # Validation du chargement de la config caméra
    cam_width = 1280
    cam_height = 720
    cam_fps = 30
    
    camera_yaml = config_dir / "camera.yaml"
    if camera_yaml.exists():
        try:
            with open(camera_yaml, 'r') as f:
                cam_conf = yaml.safe_load(f)
                rs_conf = cam_conf.get('realsense', {})
                cam_width = rs_conf.get('width', 1280)
                cam_height = rs_conf.get('height', 720)
                cam_fps = rs_conf.get('fps', 30)
                print(f"[WIZARD] Config caméra chargée: {cam_width}x{cam_height} @ {cam_fps}fps")
        except Exception as e:
            print(f"[WIZARD] Erreur chargement camera.yaml: {e}")
            
    # Configuration (peut être modifiée)
    config = {
        'proj_width': 1024,
        'proj_height': 768,
        'offset_x': 1920,
        'offset_y': 0,
        'margin': 50,
        'marker_size_m': 0.10,
        'camera_width': cam_width,
        'camera_height': cam_height,
        'camera_fps': cam_fps
    }
    
    wizard = StandaloneCalibrationWizard(config)
    calibration = wizard.run(str(config_dir))  # Passe le dossier, pas le fichier
    
    if calibration.H_CamToProj is not None:
        print("\n" + "="*50)
        print(" CALIBRATION REUSSIE")
        print(f"  Échelle: {calibration.pixels_per_meter:.2f} px/m")
        print(f"  Fichiers: calibration.npz + calibration_meta.json")
        print("="*50)
    else:
        print("\n Calibration annulée ou échouée")


if __name__ == "__main__":
    main()
