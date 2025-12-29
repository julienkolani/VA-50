"""
╔═══════════════════════════════════════════════════════════════════════════════╗
║                              ⚠️  OBSOLÈTE  ⚠️                                 ║
╠═══════════════════════════════════════════════════════════════════════════════╣
║  Ce module utilise l'ancien système de calibration.                           ║
║                                                                               ║
║  UTILISEZ PLUTÔT: standalone_wizard.py                                        ║
║    python -m perception.calibration.standalone_wizard                         ║
╚═══════════════════════════════════════════════════════════════════════════════╝

Assistant de Calibration - VERSION LEGACY (OBSOLÈTE)

Correctif :
- La boucle d'attente redessine les marqueurs ArUco au lieu de les effacer.
- Utilise la correction de distorsion optique (cv2.undistort) pour améliorer la précision aux bords.
"""

import cv2
import numpy as np
import yaml
import time
import sys
import pygame  # Import nécessaire pour les événements
from typing import Tuple, List
from ..camera.aruco_detector import ArucoDetector
from core.world.coordinate_frames import TransformManager
from .projector_display import ProjectorDisplay


class CalibrationWizard:
    def __init__(self, camera, projector_width=1024, projector_height=768, 
                 margin_px=50, monitor_offset_x=1920, monitor_offset_y=0,
                 borderless=True, hide_cursor=True, marker_size_m=0.10):
        
        self.camera = camera
        self.proj_w = projector_width
        self.proj_h = projector_height
        self.marker_size_m = marker_size_m
        
        self.aruco = ArucoDetector()
        self.transform_mgr = TransformManager()
        
        # Récupération des paramètres intrinsèques pour la correction optique
        self.K, self.D = self.camera.get_intrinsics_matrix()
        if self.K is not None:
             print("[CALIB] Correction de distorsion ACTIVE")
        else:
             print("[CALIB] ATTENTION: Pas de correction de distorsion (Intrinsics non trouvés)")

        self.projector = ProjectorDisplay(
            width=projector_width,
            height=projector_height,
            margin=margin_px,
            monitor_offset_x=monitor_offset_x,
            monitor_offset_y=monitor_offset_y,
            borderless=borderless,
            hide_cursor=hide_cursor
        )
        
        self.margin_px = margin_px
        self.arena_width_m = None
        self.arena_height_m = None
        self.H_C2W = None
        self.static_obstacles = []
    
    def _get_undistorted_frame(self):
        """Récupère une frame et applique la correction de distorsion si possible."""
        color, _ = self.camera.get_frames()
        if self.K is not None and self.D is not None and color is not None:
            color = cv2.undistort(color, self.K, self.D)
        return color

    def _wait_for_user_validation(self, message: str = "Appuyez sur ESPACE...", draw_callback=None):
        """
        Boucle d'attente générique qui supporte un redessin continu (draw_callback).
        """
        print(f"[CALIB] ATTENTE : {message}")
        
        # Si pas de callback de dessin spécifique, on affiche juste le message (fond noir)
        if draw_callback is None:
            self.projector.show_message(message, color=(255, 255, 255), bg_color=(50, 50, 50))
        
        waiting = True
        while waiting:
            # Si on a une fonction de dessin (ex: afficher les marqueurs), on l'appelle en boucle
            if draw_callback:
                draw_callback()
            
            # Gestion des événements Pygame
            events = self.projector.get_events()
            for event in events:
                if event.type == pygame.QUIT:
                    sys.exit(0)
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        waiting = False
                        print("[CALIB] Validé !")
                    elif event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                        print("[CALIB] Annulation...")
                        sys.exit(0)
            
            time.sleep(0.05) # Petite pause pour le CPU
    
    def run(self) -> dict:
        print("[CALIB] ========== Démarrage Calibration ==========")
        self.projector.start()
        
        try:
            # Étape 1 : Zone sûre (juste pour info console)
            print(f"[CALIB] MARGE définie à {self.margin_px} px")
            
            # Étape 2 : Calibration géométrique (Coins projetés)
            H_C2AV = self._step_geometric_calibration()
            
            # Étape 3 : Calibration métrique (Taille réelle)
            scale = self._step_metric_calibration(H_C2AV)
            
            # Étape 4 : Obstacles (Optionnel)
            obstacles = self._step_obstacle_mapping()
            
            # Construction des résultats
            results = {
                'projector': {
                    'width': self.proj_w,
                    'height': self.proj_h,
                    'margin': self.margin_px,
                    'margin_px': self.margin_px
                },
                'display': {
                    'fullscreen': False,
                    'display_index': 0
                },
                'arena': {
                    'width_m': self.arena_width_m,
                    'height_m': self.arena_height_m
                },
                'transform': {
                    'H_C2W': self.H_C2W.tolist() if self.H_C2W is not None else None,
                    'scale': scale,
                    'scale_m_per_av': scale
                },
                'grid': {
                    'resolution_m': 0.02,
                    'inflation_radius_m': 0.15
                },
                'obstacles': obstacles
            }
            
            print("[CALIB] Calibration terminée et sauvegardée en mémoire.")
            return results
            
        finally:
            self.projector.stop()
    
    def _step_geometric_calibration(self) -> np.ndarray:
        print("[CALIB] Étape 2/4 : Calibration géométrique")
        print("[CALIB] Affichage des 4 marqueurs ArUco (IDs 0-3)...")
        
        # On passe la fonction d'affichage en callback
        self._wait_for_user_validation(
            "Vérifiez que la caméra voit les 4 coins, puis appuyez sur ESPACE",
            draw_callback=lambda: self.projector.show_corner_markers(marker_size_px=200)
        )
        
        # Capture avec UNDISTORT
        print("[CALIB] Capture de l'image (Undistorted)...")
        color = self._get_undistorted_frame()
        detections = self.aruco.detect(color)
        
        corner_ids = [0, 1, 2, 3]
        detected_corners = {k: v for k, v in detections.items() if k in corner_ids}
        
        if len(detected_corners) != 4:
            print(f"[CALIB] ERREUR : Seulement {len(detected_corners)}/4 coins détectés !")
            print(f"[CALIB] IDs vus : {list(detected_corners.keys())}")
            raise ValueError("Échec détection des 4 coins projetés")
            
        # Calcul Homographie
        src_points = []
        dst_points = []
        
        ar = self.proj_w / self.proj_h
        av_coords = {
            0: (0.0, 0.0), 1: (ar, 0.0), 2: (ar, 1.0), 3: (0.0, 1.0)
        }
        
        for marker_id in corner_ids:
            center = detected_corners[marker_id]['center']
            src_points.append(center)
            dst_points.append(av_coords[marker_id])
            
        src_points = np.array(src_points, dtype=np.float32)
        dst_points = np.array(dst_points, dtype=np.float32)
        
        H_C2AV, _ = cv2.findHomography(src_points, dst_points)
        print("[CALIB] Matrice H_C2AV calculée.")
        return H_C2AV

    def _step_metric_calibration(self, H_C2AV: np.ndarray) -> float:
        print("[CALIB] Étape 3/4 : Calibration métrique")
        
        self._wait_for_user_validation("Placez le ROBOT (ID 4 ou 5) au centre et appuyez sur ESPACE")
        
        # Capture avec UNDISTORT
        color = self._get_undistorted_frame()
        detections = self.aruco.detect(color)
        
        marker_data = detections.get(4) or detections.get(5)
        if not marker_data:
            raise ValueError("Aucun marqueur robot (4 ou 5) trouvé !")
            
        corners_pix = marker_data['corners']
        
        # Transformation perspective
        pts_src = np.array([corners_pix], dtype=np.float32)
        pts_av = cv2.perspectiveTransform(pts_src, H_C2AV)[0]
        
        # Taille moyenne en unités virtuelles
        side_lengths = [np.linalg.norm(pts_av[j] - pts_av[(j+1)%4]) for j in range(4)]
        size_av = np.mean(side_lengths)
        
        scale_m_per_av = self.marker_size_m / size_av
        print(f"[CALIB] Échelle : 1.0 unité virtuelle = {scale_m_per_av:.4f} mètres")
        
        # Calcul final H_C2W
        self.transform_mgr.H_C2AV = H_C2AV
        self.transform_mgr.set_av_to_world_scale(scale_m_per_av)
        self.H_C2W = self.transform_mgr.H_C2W
        
        # Dimensions physiques
        ar = self.proj_w / self.proj_h
        self.arena_width_m = scale_m_per_av * ar
        self.arena_height_m = scale_m_per_av * 1.0
        
        return scale_m_per_av

    def _step_obstacle_mapping(self) -> List:
        print("[CALIB] Étape 4/4 : Cartographie")
        
        self._wait_for_user_validation(
            "Placez les obstacles, puis ESPACE (Écran deviendra BLANC)",
            draw_callback=lambda: self.projector.show_white_screen()
        )
        
        print("[CALIB] Mapping ignoré pour l'instant (retourne liste vide)")
        return []
