"""
-----------------------------------------------
Utilise l'homographie directe Caméra → Projecteur calculée par le wizard.
Charge les données de calibration depuis un fichier JSON.

Transformations disponibles:
- camera_to_projector(u, v): Pixel caméra → Pixel projecteur
- projector_to_world(px, py): Pixel projecteur → Mètres monde
- camera_to_world(u, v): Pixel caméra → Mètres monde
- world_to_projector(x, y): Mètres monde → Pixel projecteur
"""

import json
import numpy as np
import cv2
from pathlib import Path
from typing import Tuple, Optional


class UnifiedTransform:
    """
    Gestionnaire de transformations unifié.
    
    Utilise l'homographie directe H_CamToProj pour toutes les transformations.
    L'échelle pixels_per_meter permet la conversion vers le mètres.
    """
    
    def __init__(self, calibration_path: str = None):
        """
        Initialise le gestionnaire de transformations.
        
        Args:
            calibration_path: Chemin vers le fichier calibration.json
        """
        self.H_CamToProj = None      # Homographie Caméra → Projecteur
        self.H_ProjToCam = None      # Inverse: Projecteur → Caméra
        self.pixels_per_meter = 0.0
        self.proj_width = 0
        self.proj_height = 0
        self.margin = 0
        
        # Zone de jeu en pixels projecteur
        self.arena_x1 = 0
        self.arena_y1 = 0
        self.arena_x2 = 0
        self.arena_y2 = 0
        
        # Dimensions arène en mètres
        self.arena_width_m = 0.0
        self.arena_height_m = 0.0
        
        if calibration_path:
            self.load(calibration_path)
    
    def load(self, filepath: str) -> bool:
        """
        Charge les données de calibration.
        
        Supporte deux formats:
        - Nouveau (hybride): calibration.npz + calibration_meta.json
        - Ancien: calibration.json (tout en un)
        
        Args:
            filepath: Chemin vers le dossier config OU le fichier calibration.json
            
        Returns:
            True si chargement réussi
        """
        try:
            filepath = Path(filepath)
            
            # Déterminer si c'est un dossier ou un fichier
            if filepath.is_dir():
                config_path = filepath
            else:
                config_path = filepath.parent
            
            # format hybride (NPZ + JSON)
            npz_path = config_path / "calibration.npz"
            meta_path = config_path / "calibration_meta.json"
            
            if npz_path.exists() and meta_path.exists():
                return self._load_hybrid(npz_path, meta_path)
            
            # Fallback: ancien format JSON unique
            json_path = config_path / "calibration.json"
            if json_path.exists():
                return self._load_legacy_json(json_path)
            
            print(f"[TRANSFORM] ERREUR: Aucun fichier de calibration trouvé dans {config_path}")
            return False
            
        except Exception as e:
            print(f"[TRANSFORM] ERREUR chargement: {e}")
            return False
    
    def _load_hybrid(self, npz_path: Path, meta_path: Path) -> bool:
        """Charge le format  NPZ + JSON."""
        print(f"[TRANSFORM] Chargement format NPZ + JSON...")
        
        # 1. Matrices depuis NPZ
        data = np.load(npz_path)
        if 'H_CamToProj' in data:
            self.H_CamToProj = data['H_CamToProj']
            self.H_ProjToCam = np.linalg.inv(self.H_CamToProj)
        
        # 2. Métadonnées depuis JSON
        with open(meta_path, 'r') as f:
            meta = json.load(f)
        
        version = meta.get('version', '2.0')
        print(f"[TRANSFORM] Version calibration: {version}")
        
        # Projecteur
        proj = meta['projector']
        self.proj_width = proj['width']
        self.proj_height = proj['height']
        self.margin = proj['margin']
        
        # Zone de jeu
        self.arena_x1 = self.margin
        self.arena_y1 = self.margin
        self.arena_x2 = self.proj_width - self.margin
        self.arena_y2 = self.proj_height - self.margin
        
        # Échelle
        self.pixels_per_meter = meta['scale']['pixels_per_meter']
        
        # Calcul dimensions arène
        arena_width_px = self.arena_x2 - self.arena_x1
        arena_height_px = self.arena_y2 - self.arena_y1
        self.arena_width_m = arena_width_px / self.pixels_per_meter if self.pixels_per_meter > 0 else 0
        self.arena_height_m = arena_height_px / self.pixels_per_meter if self.pixels_per_meter > 0 else 0
        
        print(f"[TRANSFORM] [OK] Chargé: {self.proj_width}x{self.proj_height}, "
              f"échelle={self.pixels_per_meter:.1f} px/m, "
              f"arène={self.arena_width_m:.2f}x{self.arena_height_m:.2f}m")
        
        return True
    
    def _load_legacy_json(self, json_path: Path) -> bool:
        """Charge l'ancien format JSON unique."""
        print(f"[TRANSFORM] Chargement format legacy JSON: {json_path}")
        
        with open(json_path, 'r') as f:
            data = json.load(f)
        
        # Projecteur
        proj = data['projector']
        self.proj_width = proj['width']
        self.proj_height = proj['height']
        self.margin = proj['margin']
        
        # Zone de jeu
        self.arena_x1 = self.margin
        self.arena_y1 = self.margin
        self.arena_x2 = self.proj_width - self.margin
        self.arena_y2 = self.proj_height - self.margin
        
        # Homographie
        H_list = data['homography']['H_CamToProj']
        if H_list:
            self.H_CamToProj = np.array(H_list, dtype=np.float32)
            self.H_ProjToCam = np.linalg.inv(self.H_CamToProj)
        
        # Échelle
        self.pixels_per_meter = data['scale']['pixels_per_meter']
        
        # Calcul dimensions arène
        arena_width_px = self.arena_x2 - self.arena_x1
        arena_height_px = self.arena_y2 - self.arena_y1
        self.arena_width_m = arena_width_px / self.pixels_per_meter if self.pixels_per_meter > 0 else 0
        self.arena_height_m = arena_height_px / self.pixels_per_meter if self.pixels_per_meter > 0 else 0
        
        print(f"[TRANSFORM] [OK] Chargé (legacy): {self.proj_width}x{self.proj_height}, "
              f"échelle={self.pixels_per_meter:.1f} px/m")
        
        return True
    
    def is_calibrated(self) -> bool:
        """Vérifie si la calibration est chargée."""
        return self.H_CamToProj is not None and self.pixels_per_meter > 0
    
    # =========================================================================
    # TRANSFORMATIONS PRINCIPALES
    # =========================================================================
    
    def camera_to_projector(self, u: float, v: float) -> Tuple[float, float]:
        """
        Transforme un pixel caméra vers un pixel projecteur.
        
        Args:
            u, v: Coordonnées pixel caméra
            
        Returns:
            (px, py): Coordonnées pixel projecteur
        """
        if self.H_CamToProj is None:
            raise ValueError("Calibration non chargée")
        
        p_cam = np.array([[[u, v]]], dtype=np.float32)
        p_proj = cv2.perspectiveTransform(p_cam, self.H_CamToProj)[0][0]
        return float(p_proj[0]), float(p_proj[1])
    
    def projector_to_camera(self, px: float, py: float) -> Tuple[float, float]:
        """
        Transforme un pixel projecteur vers un pixel caméra.
        
        Args:
            px, py: Coordonnées pixel projecteur
            
        Returns:
            (u, v): Coordonnées pixel caméra
        """
        if self.H_ProjToCam is None:
            raise ValueError("Calibration non chargée")
        
        p_proj = np.array([[[px, py]]], dtype=np.float32)
        p_cam = cv2.perspectiveTransform(p_proj, self.H_ProjToCam)[0][0]
        return float(p_cam[0]), float(p_cam[1])
    
    def projector_to_world(self, px: float, py: float) -> Tuple[float, float]:
        """
        Transforme un pixel projecteur vers des coordonnées monde (mètres).
        
        Origine: coin haut-gauche de la zone de jeu
        X: vers la droite
        Y: vers le bas (convention écran) OU vers le haut (convention robot)
        
        Args:
            px, py: Coordonnées pixel projecteur
            
        Returns:
            (x, y): Coordonnées en mètres (Y inversé pour convention robot)
        """
        if self.pixels_per_meter <= 0:
            raise ValueError("Échelle non définie")
        
        # Relatif à l'origine de la zone de jeu
        dx = px - self.arena_x1
        dy = py - self.arena_y1
        
        # Conversion en mètres
        x = dx / self.pixels_per_meter
        y = dy / self.pixels_per_meter
        
        # Convention robot: Y vers le haut (inverser si nécessaire)
        # y = self.arena_height_m - y
        
        return x, y
    
    def world_to_projector(self, x: float, y: float) -> Tuple[int, int]:
        """
        Transforme des coordonnées monde (mètres) vers un pixel projecteur.
        
        Args:
            x, y: Coordonnées en mètres
            
        Returns:
            (px, py): Coordonnées pixel projecteur
        """
        if self.pixels_per_meter <= 0:
            raise ValueError("Échelle non définie")
        
        # Conversion en pixels
        dx = x * self.pixels_per_meter
        dy = y * self.pixels_per_meter
        
        # Position absolue
        px = int(self.arena_x1 + dx)
        py = int(self.arena_y1 + dy)
        
        return px, py
    
    def camera_to_world(self, u: float, v: float) -> Tuple[float, float]:
        """
        Transforme un pixel caméra directement vers des coordonnées monde (mètres).
        
        Args:
            u, v: Coordonnées pixel caméra
            
        Returns:
            (x, y): Coordonnées en mètres
        """
        px, py = self.camera_to_projector(u, v)
        return self.projector_to_world(px, py)
    
    def world_to_camera(self, x: float, y: float) -> Tuple[float, float]:
        """
        Transforme des coordonnées monde vers un pixel caméra.
        
        Args:
            x, y: Coordonnées en mètres
            
        Returns:
            (u, v): Coordonnées pixel caméra
        """
        px, py = self.world_to_projector(x, y)
        return self.projector_to_camera(float(px), float(py))
    
    # =========================================================================
    # TRANSFORMATIONS PAR LOT
    # =========================================================================
    
    def batch_camera_to_projector(self, points_cam: np.ndarray) -> np.ndarray:
        """
        Transforme plusieurs points caméra vers projecteur.
        
        Args:
            points_cam: Tableau Nx2 de coordonnées caméra
            
        Returns:
            Tableau Nx2 de coordonnées projecteur
        """
        if self.H_CamToProj is None:
            raise ValueError("Calibration non chargée")
        
        points_cam = np.array(points_cam, dtype=np.float32)
        if points_cam.ndim == 2:
            points_cam = points_cam.reshape(1, -1, 2)
        
        return cv2.perspectiveTransform(points_cam, self.H_CamToProj)[0]
    
    def batch_camera_to_world(self, points_cam: np.ndarray) -> np.ndarray:
        """
        Transforme plusieurs points caméra vers monde (mètres).
        
        Args:
            points_cam: Tableau Nx2 de coordonnées caméra
            
        Returns:
            Tableau Nx2 de coordonnées monde
        """
        points_proj = self.batch_camera_to_projector(points_cam)
        
        # Conversion en mètres
        points_world = np.zeros_like(points_proj)
        points_world[:, 0] = (points_proj[:, 0] - self.arena_x1) / self.pixels_per_meter
        points_world[:, 1] = (points_proj[:, 1] - self.arena_y1) / self.pixels_per_meter
        
        return points_world
    
    # =========================================================================
    # WARP IMAGE
    # =========================================================================
    
    def warp_camera_to_projector(self, image: np.ndarray) -> np.ndarray:
        """
        Warpe une image caméra vers l'espace projecteur.
        
        Args:
            image: Image BGR de la caméra
            
        Returns:
            Image warpée aux dimensions du projecteur
        """
        if self.H_CamToProj is None:
            raise ValueError("Calibration non chargée")
        
        return cv2.warpPerspective(
            image, 
            self.H_CamToProj, 
            (self.proj_width, self.proj_height)
        )
    
    # =========================================================================
    # UTILITAIRES
    # =========================================================================
    
    def get_arena_bounds_m(self) -> Tuple[float, float, float, float]:
        """
        Retourne les limites de l'arène en mètres.
        
        Returns:
            (x_min, y_min, x_max, y_max) en mètres
        """
        return 0.0, 0.0, self.arena_width_m, self.arena_height_m
    
    def get_arena_bounds_px(self) -> Tuple[int, int, int, int]:
        """
        Retourne les limites de l'arène en pixels projecteur.
        
        Returns:
            (x1, y1, x2, y2) en pixels
        """
        return self.arena_x1, self.arena_y1, self.arena_x2, self.arena_y2
    
    def meters_to_pixels(self, distance_m: float) -> float:
        """Convertit une distance en mètres vers pixels."""
        return distance_m * self.pixels_per_meter
    
    def pixels_to_meters(self, distance_px: float) -> float:
        """Convertit une distance en pixels vers mètres."""
        return distance_px / self.pixels_per_meter if self.pixels_per_meter > 0 else 0


# Fonction de compatibilité pour charger facilement
def load_calibration(config_dir: str = None) -> UnifiedTransform:
    """
    Charge la calibration depuis le fichier par défaut.
    
    Args:
        config_dir: Dossier de configuration (optionnel)
        
    Returns:
        UnifiedTransform initialisé
    """
    if config_dir is None:
        # Chemin par défaut relatif au module
        config_dir = Path(__file__).parent.parent.parent / "config"
    else:
        config_dir = Path(config_dir)
    
    # Vérification: soit le legacy JSON, soit le hybrid (npz + meta)
    legacy_path = config_dir / "calibration.json"
    hybrid_flag = (config_dir / "calibration.npz").exists() and (config_dir / "calibration_meta.json").exists()
    
    if not legacy_path.exists() and not hybrid_flag:
        print(f"[TRANSFORM] ATTENTION: Aucune calibration trouvée dans {config_dir}")
        print("[TRANSFORM] Lancez d'abord: python -m perception.calibration.standalone_wizard")
        return UnifiedTransform()
    
    # On passe le DOSSIER au constructeur, qui saura quoi chercher
    return UnifiedTransform(str(config_dir))
