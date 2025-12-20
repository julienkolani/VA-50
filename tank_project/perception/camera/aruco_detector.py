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
    
    Utilise cv2.aruco pour la détection de marqueurs.
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
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        
        self.marker_size_m = marker_size_m
        
    def detect(self, image: np.ndarray) -> Dict[int, Dict]:
        """
        Détecte les marqueurs ArUco dans l'image.
        
        Args:
            image: Image d'entrée (BGR ou niveaux de gris)
            
        Returns:
            dict: {
                marker_id: {
                    'center': (u, v),  # coordonnées pixel
                    'corners': [(u1,v1), (u2,v2), (u3,v3), (u4,v4)],
                    'orientation': theta  # radians
                }
            }
            
        Logs:
            [ARUCO] Détecté N marqueurs : [IDs]
        """
        # Convertit en niveaux de gris si nécessaire
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # Détecte les marqueurs
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        results = {}
        
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                marker_corners = corners[i][0]  # Shape: (4, 2)
                
                # Calcule le centre
                center = marker_corners.mean(axis=0)
                
                # Calcule l'orientation (du coin 0 au coin 1)
                # Ordre des coins : haut-gauche, haut-droite, bas-droite, bas-gauche
                dx = marker_corners[1][0] - marker_corners[0][0]
                dy = marker_corners[1][1] - marker_corners[0][1]
                orientation = np.arctan2(dy, dx)
                
                results[marker_id] = {
                    'center': tuple(center),
                    'corners': [tuple(c) for c in marker_corners],
                    'orientation': orientation
                }
            
            print("[ARUCO] Détecté {} marqueurs : {}".format(len(ids), ids.flatten().tolist()))
        
        return results
    
    def estimate_marker_size_av(self, marker_corners, H_C2AV):
        """
        Estime la taille du marqueur en unités de l'Arène Virtuelle.
        
        Utilisé pendant l'étalonnage pour calculer l'échelle métrique.
        
        Args:
            marker_corners: Liste des 4 positions de coins en pixels
            H_C2AV: Homographie caméra -> arène virtuelle
            
        Returns:
            float: Longueur du côté du marqueur en unités AV
        """
        # Transforme les coins vers l'espace AV
        corners_av = []
        for u, v in marker_corners:
            p_cam = np.array([u, v, 1.0])
            p_av = H_C2AV @ p_cam
            p_av = p_av[:2] / p_av[2]  # Normalize
            corners_av.append(p_av)
        
        # Calcule la longueur moyenne des côtés
        corners_av = np.array(corners_av)
        side_lengths = []
        for i in range(4):
            j = (i + 1) % 4
            length = np.linalg.norm(corners_av[j] - corners_av[i])
            side_lengths.append(length)
        
        avg_length = np.mean(side_lengths)
        
        return avg_length
    
    def draw_detections(self, image: np.ndarray, detections: Dict) -> np.ndarray:
        """
        Dessine les marqueurs détectés sur l'image (pour débogage).
        
        Args:
            image: Image d'entrée
            detections: Résultats de détection de detect()
            
        Returns:
            Image avec marqueurs dessinés
        """
        img_draw = image.copy()
        
        for marker_id, data in detections.items():
            center = data['center']
            corners = data['corners']
            
            # Dessine les coins
            corners_array = np.array(corners, dtype=np.int32)
            cv2.polylines(img_draw, [corners_array], True, (0, 255, 0), 2)
            
            # Dessine le centre
            cv2.circle(img_draw, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)
            
            # Dessine l'ID
            cv2.putText(img_draw, f"ID:{marker_id}", 
                       (int(center[0]), int(center[1]) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        return img_draw
