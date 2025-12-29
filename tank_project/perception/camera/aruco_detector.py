"""
Détecteur ArUco - Détection de Marqueurs & Estimation de Pose

Détecte les marqueurs ArUco dans les images caméra :
- Marqueurs projetés (ID 0-3) : coins de l'arène pour l'étalonnage
- Marqueurs robots (ID 4, 5) : suivi des robots

Fournit :
- Positions centrales des marqueurs (pixels)
- Orientations des marqueurs (radians)
- Positions des coins pour l'estimation de l'échelle

Logs : [ARUCO] Détecté N marqueurs : [IDs]
"""

import cv2
import numpy as np
from typing import List, Dict, Tuple, Optional


class ArucoDetector:
    """
    Détection de marqueurs ArUco et estimation de pose.
    Utilise cv2.aruco pour la détection et cv2.solvePnP pour la pose 3D.
    """
    
    def __init__(self, 
                 dictionary_type=cv2.aruco.DICT_4X4_50,
                 marker_size_m: float = 0.10):
        """
        Initialise le détecteur ArUco.
        
        Args:
            dictionary_type: Dictionnaire ArUco (défaut : 4x4, 50 marqueurs)
            marker_size_m: Taille physique du marqueur en mètres (pour estimation échelle)
        """
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_type)
        self.parameters = cv2.aruco.DetectorParameters()
        
        # Compatibilité OpenCV: utilise ArucoDetector (4.7+) avec fallback vers API legacy
        try:
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
            self.use_legacy_api = False
        except AttributeError:
            # OpenCV < 4.7.0 n'a pas ArucoDetector, utiliser l'API legacy
            self.detector = None
            self.use_legacy_api = True
            print("[ARUCO] Utilisation de l'API legacy (OpenCV < 4.7)")
        
        self.marker_size_m = marker_size_m
        
        # Pré-calcul des points objets 3D pour un marqueur (centré à 0,0,0)
        # Ordre: Haut-Gauche, Haut-Droite, Bas-Droite, Bas-Gauche (Sens horaire)
        half_size = marker_size_m / 2.0
        self.obj_points = np.array([
            [-half_size, half_size, 0],
            [half_size, half_size, 0],
            [half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)
        
    def detect(self, image: np.ndarray) -> Dict[int, Dict]:
        """
        Détecte les marqueurs ArUco dans l'image.
        """
        # Convertit en niveaux de gris si nécessaire
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # Détecte les marqueurs (API moderne ou legacy selon version OpenCV)
        if self.use_legacy_api:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
        else:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        
        results = {}
        
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                marker_corners = corners[i][0]  # Shape: (4, 2)
                
                # Calcule le centre
                center = marker_corners.mean(axis=0)
                
                # Calcule l'orientation (du coin 0 au coin 1)
                dx = marker_corners[1][0] - marker_corners[0][0]
                dy = marker_corners[1][1] - marker_corners[0][1]
                orientation = np.arctan2(dy, dx)
                
                results[marker_id] = {
                    'center': tuple(center),
                    'corners': [tuple(c) for c in marker_corners],
                    'orientation': orientation
                }
            
            # Décommentez pour debug verbeux si besoin
            # print("[ARUCO] Détecté {} marqueurs : {}".format(len(ids), ids.flatten().tolist()))
        
        return results
    
    def estimate_marker_size_av(self, marker_corners, H_C2AV):
        """
        Estime la taille du marqueur en unités de l'Arène Virtuelle.
        Utilisé pendant l'étalonnage.
        """
        corners_av = []
        for u, v in marker_corners:
            p_cam = np.array([u, v, 1.0])
            p_av = H_C2AV @ p_cam
            p_av = p_av[:2] / p_av[2]
            corners_av.append(p_av)
        
        corners_av = np.array(corners_av)
        side_lengths = []
        for i in range(4):
            j = (i + 1) % 4
            length = np.linalg.norm(corners_av[j] - corners_av[i])
            side_lengths.append(length)
        
        return np.mean(side_lengths)
    
    def get_corrected_pose(self, corners: np.ndarray, mtx: np.ndarray, dist: np.ndarray, 
                          obj_height_m: float = 0.30) -> Tuple[Tuple[float, float], float]:
        """
        Calcule la position corrigée de la parallaxe.
        Remplace estimatePoseSingleMarkers par solvePnP (plus robuste).
        """
        # Formater les coins image pour solvePnP (doit être float32)
        # corners arrive souvent en shape (1, 4, 2) ou (4, 2)
        img_points = corners.reshape(4, 2).astype(np.float32)
        
        # 1. Estimation de Pose (PnP)
        # Trouve la rotation et translation du marqueur par rapport à la caméra
        success, rvec, tvec = cv2.solvePnP(self.obj_points, img_points, mtx, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        
        if not success:
            # Fallback si PnP échoue
            center = img_points.mean(axis=0)
            return (center[0], center[1]), 2.5
            
        # Z est la distance caméra <-> marqueur (composante Z du vecteur translation)
        z_camera = float(tvec[2])
        
        # Sécurité division par zéro
        if z_camera < 0.1: z_camera = 0.1
            
        # 2. Facteur de correction (Thalès)
        correction_factor = 1.0 - (obj_height_m / z_camera)
        
        # 3. Correction géométrique vers le centre optique
        cx, cy = mtx[0, 2], mtx[1, 2]
        
        # Centre détecté (brut)
        u_raw, v_raw = img_points.mean(axis=0)
        
        # Vecteur radial
        dx = u_raw - cx
        dy = v_raw - cy
        
        # Application
        u_corr = cx + dx * correction_factor
        v_corr = cy + dy * correction_factor
        
        return (u_corr, v_corr), z_camera

    def draw_detections(self, image: np.ndarray, detections: Dict) -> np.ndarray:
        """Dessine les marqueurs détectés sur l'image."""
        img_draw = image.copy()
        for marker_id, data in detections.items():
            center = data['center']
            corners = data['corners']
            corners_array = np.array(corners, dtype=np.int32)
            cv2.polylines(img_draw, [corners_array], True, (0, 255, 0), 2)
            cv2.circle(img_draw, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)
            cv2.putText(img_draw, f"ID:{marker_id}", 
                       (int(center[0]), int(center[1]) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        return img_draw
