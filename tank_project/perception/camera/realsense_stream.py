"""
Flux RealSense - Interface Caméra Intel RealSense

Gère la caméra RealSense D435/D455 :
- Acquisition flux couleur
- Flux profondeur (optionnel)
- Configuration caméra
- Gestion fréquence d'images

Fournit frames couleur et profondeur synchronisées à 30 FPS.

Logs : préfixe [REALSENSE] pour les opérations caméra
"""

import pyrealsense2 as rs
import numpy as np
from typing import Tuple, Optional


class RealSenseStream:
    """
    Interface pour caméra Intel RealSense.
    
    Gère l'initialisation de la caméra et l'acquisition des frames.
    """
    
    def __init__(self, 
                 width: int = 1280, 
                 height: int = 720, 
                 fps: int = 30,
                 enable_depth: bool = False):
        """
        Initialise la caméra RealSense.
        
        Args:
            width: Largeur frame
            height: Hauteur frame
            fps: Fréquence d'images
            enable_depth: Activer flux profondeur
            
        Logs:
            [REALSENSE] Camera initialized: WxH @ FPS fps
        """
        self.width = width
        self.height = height
        self.fps = fps
        self.enable_depth = enable_depth
        
        self.pipeline = None
        self.config = None
        
    def start(self):
        """
        Démarre le pipeline caméra.
        
        Logs:
            [REALSENSE] Pipeline démarré
            [REALSENSE] Échec du démarrage : erreur
        """
        try:
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Configure les flux
            self.config.enable_stream(rs.stream.color, 
                                     self.width, self.height, 
                                     rs.format.bgr8, self.fps)
            
            if self.enable_depth:
                self.config.enable_stream(rs.stream.depth, 
                                         self.width, self.height, 
                                         rs.format.z16, self.fps)
            
            # Démarre pipeline
            self.pipeline.start(self.config)
            
            print("[REALSENSE] Pipeline démarré : {}x{} @ {} fps".format(self.width, self.height, self.fps))
            
        except Exception as e:
            print("[REALSENSE] Échec du démarrage : {}".format(e))
            raise
    
    def get_intrinsics_matrix(self):
        """
        Retourne la matrice de caméra (K) et les coefficients de distorsion (D).
        """
        if self.pipeline:
            try:
                profile = self.pipeline.get_active_profile()
                color_stream = profile.get_stream(rs.stream.color)
                intr = color_stream.as_video_stream_profile().get_intrinsics()
                
                # Matrice K (3x3)
                K = np.array([
                    [intr.fx, 0, intr.ppx],
                    [0, intr.fy, intr.ppy],
                    [0, 0, 1]
                ])
                
                # Coefficients D (Distortion)
                D = np.array(intr.coeffs)
                
                return K, D
            except Exception as e:
                print(f"[REALSENSE] Erreur récupération intrinsics : {e}")
                return None, None
        return None, None

    def stop(self):
        """Arrête le pipeline caméra."""
        if self.pipeline:
            self.pipeline.stop()
            print("[REALSENSE] Pipeline arrêté")
    
    def get_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Obtient les dernières frames couleur et profondeur.
        
        Returns:
            (color_frame, depth_frame): tableaux numpy
            color_frame: Image BGR HxWx3
            depth_frame: Carte profondeur HxW (mm) ou None
        """
        if not self.pipeline:
            return (None, None)
        
        try:
            # Attend les frames
            frames = self.pipeline.wait_for_frames()
            
            # Obtient frame couleur
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data()) if color_frame else None
            
            # Obtient frame profondeur (si activé)
            depth_image = None
            if self.enable_depth:
                depth_frame = frames.get_depth_frame()
                depth_image = np.asanyarray(depth_frame.get_data()) if depth_frame else None
            
            return (color_image, depth_image)
            
        except Exception as e:
            print("[REALSENSE] Erreur d'acquisition frame : {}".format(e))
            return (None, None)
    
    def get_intrinsics(self):
        """
        Obtient les paramètres intrinsèques de la caméra.
        
        Returns:
            objet rs.intrinsics avec fx, fy, cx, cy
        """
        if self.pipeline:
            profile = self.pipeline.get_active_profile()
            color_stream = profile.get_stream(rs.stream.color)
            intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
            return intrinsics
        return None
