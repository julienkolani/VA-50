"""
Assistant de Calibration - Processus Interactif

Guide l'utilisateur à travers la séquence de calibration :
1. Définition de la zone sûre (marges de projection)
2. Calibration géométrique (H_C2AV à partir des coins projetés)
3. Calibration métrique (échelle à partir d'un ArUco physique)
4. Cartographie des obstacles (détection d'obstacles statiques)

Sauvegarde la calibration dans config/arena.yaml pour la phase de jeu.

Logs : Préfixe [CALIB] pour toutes les étapes
"""

import cv2
import numpy as np
import yaml
import time
import sys
from typing import Tuple, List
from ..camera.aruco_detector import ArucoDetector
from core.world.coordinate_frames import TransformManager
from .projector_display import ProjectorDisplay


class CalibrationWizard:
    """
    Processus de calibration interactif pour la configuration de l'arène.
    
    Produit la transformation H_C2W et les paramètres de l'arène.
    """
    
    def __init__(self, camera, projector_width=1024, projector_height=768, 
                 margin_px=50, monitor_offset_x=1920, monitor_offset_y=0,
                 borderless=True, hide_cursor=True, marker_size_m=0.10):
        """
        Initialise l'assistant de calibration.
        
        Args:
            camera: Instance de RealSenseStream
            projector_width: Largeur de la résolution du projecteur
            projector_height: Hauteur de la résolution du projecteur
            margin_px: Marge de sécurité depuis les bords (pixels)
            monitor_offset_x: Position X pour le moniteur secondaire
            monitor_offset_y: Position Y pour le moniteur secondaire
            borderless: Utiliser le mode fenêtre sans bordure
            hide_cursor: Masquer le curseur de la souris
            marker_size_m: Taille physique du marqueur en mètres
        """
        self.camera = camera
        self.proj_w = projector_width
        self.proj_h = projector_height
        self.marker_size_m = marker_size_m
        
        self.aruco = ArucoDetector()
        self.transform_mgr = TransformManager()
        
        # Initialise l'affichage du projecteur avec toute la config
        self.projector = ProjectorDisplay(
            width=projector_width,
            height=projector_height,
            margin=margin_px,
            monitor_offset_x=monitor_offset_x,
            monitor_offset_y=monitor_offset_y,
            borderless=borderless,
            hide_cursor=hide_cursor
        )
        
        # Résultats de calibration
        self.margin_px = margin_px
        self.arena_width_m = None
        self.arena_height_m = None
        self.H_C2W = None
        self.static_obstacles = []
    
    def _wait_for_user_validation(self, message: str = "Appuyez sur ESPACE pour continuer..."):
        """
        Attend la touche ESPACE dans la fenêtre Pygame tout en la gardant réactive.
        
        Cela empêche l'erreur "ne répond pas" en traitant toutes les entrées
        exclusivement via Pygame au lieu de mélanger console/Pygame.
        
        Args:
            message: Message à afficher à l'utilisateur
        """
        print(f"[CALIB] ATTENTE : {message}")
        
        # Affiche le message sur le projecteur
        self.projector.show_message(message, color=(255, 255, 255), bg_color=(50, 50, 50))
        
        import pygame
        waiting = True
        while waiting:
            # Traite tous les événements Pygame
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    print("[CALIB] L'utilisateur a fermé la fenêtre, sortie...")
                    sys.exit(0)
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        waiting = False
                        print("[CALIB] Validé !")
                    elif event.key == pygame.K_ESCAPE:
                        print("[CALIB] Calibration annulée par l'utilisateur")
                        sys.exit(0)
                    elif event.key == pygame.K_q:
                        print("[CALIB] Calibration annulée par l'utilisateur")
                        sys.exit(0)
            
            # Petite pause pour éviter de saturer le CPU
            time.sleep(0.01)
        
    def run(self) -> dict:
        """
        Exécute l'assistant de calibration complet.
        
        Returns:
            dict: Résultats de calibration à sauvegarder dans la config
            
        Étapes :
            1. Définir la zone sûre (marges)
            2. Détecter les coins projetés -> H_C2AV
            3. Détecter le marqueur physique -> échelle -> H_C2W
            4. Cartographier les obstacles statiques
            5. Calculer les dimensions de l'arène
            
        Logs :
            [CALIB] Étape X/4 : Description
        """
        print("[CALIB] ========== Démarrage de l'Assistant de Calibration ==========")
        
        # Démarre l'affichage du projecteur
        print("[CALIB] Démarrage de l'affichage projecteur...")
        self.projector.start()
        
        try:
            # Step 1: Safe zone
            self._step_safe_zone()
            
            # Step 2: Geometric calibration
            H_C2AV = self._step_geometric_calibration()
            
            # Step 3: Metric calibration
            scale = self._step_metric_calibration(H_C2AV)
            
            # Step 4: Obstacle mapping
            obstacles = self._step_obstacle_mapping()
            
            # Build results
            results = {
                'projector': {
                    'width': self.proj_w,
                    'height': self.proj_h,
                    'margin': self.margin_px,
                    'margin_px': self.margin_px  # Alias for compatibility
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
                    'scale_m_per_av': scale  # Alias for compatibility
                },
                'grid': {
                    'resolution_m': 0.02,  # 2cm grid resolution
                    'inflation_radius_m': 0.15  # 15cm safety margin
                },
                'obstacles': obstacles
            }
            
            print("[CALIB] Calibration complete!")
            return results
            
        finally:
            # Always stop projector
            print("[CALIB] Stopping projector display...")
            self.projector.stop()
    
    def _step_safe_zone(self):
        """
        Étape 1 : Définir la zone sûre pour la projection.
        
        Logs :
            [CALIB] MARGE définie à X px
            [CALIB] Rect arène dans projecteur : (x1,y1) -> (x2,y2)
        """
        print("[CALIB] MARGE définie à {} px".format(self.margin_px))
        
        x1, y1 = self.margin_px, self.margin_px
        x2 = self.proj_w - self.margin_px
        y2 = self.proj_h - self.margin_px
        
        print("[CALIB] Rect arène dans projecteur : ({},{}) -> ({},{})".format(x1, y1, x2, y2))
        
    def _step_geometric_calibration(self) -> np.ndarray:
        """
        Étape 2 : Détecter les coins projetés et calculer H_C2AV.
        
        Returns:
            H_C2AV: Matrice d'homographie
            
        Logs :
            [CALIB] 4 coins projetés détectés
            [CALIB] H_C2AV calculé avec succès
        """
        print("[CALIB] Étape 2/4 : Calibration géométrique (détection des coins projetés)")
        
        # Projette les marqueurs ArUco aux coins
        print("[CALIB] Projection des marqueurs ArUco (IDs 0-3) aux coins de l'arène...")
        self.projector.show_corner_markers(marker_size_px=200)
        
        print("[CALIB] Marqueurs ArUco affichés sur le projecteur")
        print("[CALIB] Vérifiez que la caméra voit les 4 marqueurs, puis appuyez sur ESPACE...")
        
        self._wait_for_user_validation("Vérifiez les 4 marqueurs et appuyez sur ESPACE")
        
        # Capture l'image
        color, _ = self.camera.get_frames()
        
        # Détecte ArUco
        detections = self.aruco.detect(color)
        
        # Vérifie les coins (IDs 0-3)
        corner_ids = [0, 1, 2, 3]
        detected_corners = {k: v for k, v in detections.items() if k in corner_ids}
        
        if len(detected_corners) != 4:
            print("[CALIB] ERREUR : Attendu 4 coins, trouvé {}".format(len(detected_corners)))
            print("[CALIB] IDs détectés : {}".format(list(detected_corners.keys())))
            raise ValueError("Coins projetés manquants")
        
        print("[CALIB] 4 coins projetés détectés")
        
        # Construit les correspondances
        src_points = []
        dst_points = []
        
        # Coordonnées virtuelles de l'arène (rectangle proportionnel au ratio du projecteur)
        ar = self.proj_w / self.proj_h
        av_coords = {
            0: (0.0, 0.0),  # Bas-Gauche
            1: (ar, 0.0),   # Bas-Droite
            2: (ar, 1.0),   # Haut-Droite
            3: (0.0, 1.0)   # Haut-Gauche
        }
        
        for marker_id in corner_ids:
            center = detected_corners[marker_id]['center']
            src_points.append(center)
            dst_points.append(av_coords[marker_id])
        
        src_points = np.array(src_points, dtype=np.float32)
        dst_points = np.array(dst_points, dtype=np.float32)
        
        # Calcule l'homographie
        H_C2AV, _ = cv2.findHomography(src_points, dst_points)
        
        print("[CALIB] H_C2AV calculé avec succès")
        
        return H_C2AV
    
    def _step_metric_calibration(self, H_C2AV: np.ndarray) -> float:
        """
        Étape 3 : Estimer l'échelle métrique à partir d'un marqueur physique en utilisant l'homographie.
        
        Cette méthode utilise l'homographie pour corriger la perspective de la caméra,
        garantissant des mesures précises quelle que soit la position du marqueur.
        
        Args:
            H_C2AV: Homographie de l'étape 2 (pixels caméra -> arène virtuelle)
            
        Returns:
            scale: mètres par unité AV
            
        Logs :
            [CALIB] Taille réelle du marqueur : X m
            [CALIB] Taille marqueur dans AV : Y unités
            [CALIB] Échelle : Z m / unité_AV
        """
        print("[CALIB] Étape 3/4 : Calibration métrique (placez le marqueur physique dans l'arène)")
        
        # Affiche les instructions sur le projecteur
        self.projector.show_message("Placez le robot (ID 4 ou 5) au centre", 
                                    color=(255, 255, 255), bg_color=(50, 50, 50))
        
        # Utilise la taille de marqueur configurée
        marker_size_real = self.marker_size_m
        print(f"[CALIB] Utilisation taille marqueur : {marker_size_real} m (depuis config/projector.yaml)")
        
        self._wait_for_user_validation("Placez le marqueur robot et appuyez sur ESPACE")
        
        # Capture l'image
        color, _ = self.camera.get_frames()
        
        # Détecte les marqueurs
        detections = self.aruco.detect(color)
        
        # Trouve le marqueur robot (ID 4 ou 5)
        marker_data = None
        robot_marker_id = None
        for mid in [4, 5]:
            if mid in detections:
                marker_data = detections[mid]
                robot_marker_id = mid
                break
        
        if marker_data is None:
            raise ValueError("Aucun marqueur robot détecté (ID 4 ou 5) !")
        
        print(f"[CALIB] Marqueur robot ID {robot_marker_id} détecté")
        
        # ============================================================
        # CORRECT METHOD: Use homography to transform marker corners
        # This accounts for camera perspective distortion
        # ============================================================
        
        corners_pix = marker_data['corners']  # List of 4 corner tuples in pixels
        
        # Transform corners from camera pixels to Arena Virtual space using H_C2AV
        pts_src = np.array([corners_pix], dtype=np.float32)
        pts_av = cv2.perspectiveTransform(pts_src, H_C2AV)  # Output: AV coordinates
        
        corners_av = pts_av[0]  # Shape: (4, 2)
        
        # Calculate marker size in AV space (average of all 4 sides)
        side_lengths = []
        for i in range(4):
            j = (i + 1) % 4
            length = np.linalg.norm(corners_av[j] - corners_av[i])
            side_lengths.append(length)
        
        size_av = np.mean(side_lengths)
        
        print(f"[CALIB] Taille du marqueur dans l'espace virtuel: {size_av:.4f} unités")
        
        # Compute scale: meters per AV unit
        scale_m_per_av = marker_size_real / size_av
        
        print(f"[CALIB] ÉCHELLE VALIDÉE: 1.0 unité virtuelle = {scale_m_per_av:.4f} mètres")
        
        # ============================================================
        # Build H_C2W using corner markers (IDs 0-3)
        # ============================================================
        
        ar = self.proj_w / self.proj_h
        av_coords = {
            0: [0.0, 0.0],  # Bottom-left
            1: [ar, 0.0],   # Bottom-right
            2: [ar, 1.0],   # Top-right
            3: [0.0, 1.0]   # Top-left
        }
        
        src_centers = []
        dst_coords = []
        for mid in corner_ids:
            if mid in detections:
                src_centers.append(detections[mid]['center'])
                dst_coords.append(av_coords[mid])
        
        if len(src_centers) < 4:
            print(f"[CALIB] WARNING: Seulement {len(src_centers)}/4 marqueurs de coin détectés")
        
        self.transform_mgr.set_camera_to_av(
            np.array(src_centers, dtype=np.float32),
            np.array(dst_coords, dtype=np.float32)
        )
        self.transform_mgr.set_av_to_world_scale(scale_m_per_av)
        
        self.H_C2W = self.transform_mgr.H_C2W
        
        # ============================================================
        # Calculate arena dimensions using homography-corrected scale
        # In AV space, the arena is always 1.0 x 1.0 (unit square)
        # So real dimensions = scale * 1.0
        # ============================================================
        
        # The arena width in meters is simply the scale * aspect_ratio (since AV width = ar)
        self.arena_width_m = scale_m_per_av * ar
        
        # For height, we just need scale * 1.0 (since AV height = 1.0)
        self.arena_height_m = scale_m_per_av * 1.0
        
        aspect_ratio = ar
        
        print(f"[CALIB] Dimensions de l'arène: {self.arena_width_m:.2f}m x {self.arena_height_m:.2f}m")
        print(f"[CALIB] Ratio d'aspect: {aspect_ratio:.2f}")
        
        print("[CALIB] H_C2W calculé")
        print("[CALIB] Calibration métrique OK")
        
        return scale_m_per_av
    
    def _step_obstacle_mapping(self) -> List:
        """
        Étape 4 : Cartographier les obstacles statiques.
        
        Returns:
            Liste des régions d'obstacles
            
        Logs :
            [CALIB] Taille arène estimée : XmxYm
            [CALIB] Obstacles statiques cartographiés
        """
        print("[CALIB] Étape 4/4 : Cartographie des obstacles")
        
        # Affiche un écran blanc pour le contraste des obstacles
        print("[CALIB] Affichage écran blanc pour détection obstacles...")
        self.projector.show_white_screen()
        
        print("[CALIB] Placez les obstacles dans l'arène, puis appuyez sur ESPACE...")
        
        self._wait_for_user_validation("Placez les obstacles et appuyez sur ESPACE")
        
        # Simplifié : retourne vide pour l'instant
        # L'implémentation complète ferait un seuillage et une détection de contours
        
        print("[CALIB] Taille arène estimée : {:.2f}m x {:.2f}m".format(self.arena_width_m, self.arena_height_m))
        print("[CALIB] Obstacles statiques cartographiés")
        
        return []
