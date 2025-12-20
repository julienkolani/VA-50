"""
Homography - Calculs Transformations Homographiques

Calcule et applique les homographies:
- H_C2AV: Caméra -> Arène Virtuelle
- H_AV2W: Arène Virtuelle -> Monde (scaling)
- H_C2W: Caméra -> Monde (combinée)

Utilisé par calibration_wizard et coordinate_frames.

Logs: [HOMOGRAPHY] prefix
"""

import cv2
import numpy as np
from typing import List, Tuple


def compute_homography(src_points: np.ndarray,
                      dst_points: np.ndarray) -> np.ndarray:
    """
    Calcule homographie entre points source et destination.
    
    Args:
        src_points: Points source (Nx2) en pixels caméra
        dst_points: Points destination (Nx2) en coordonnées cible
        
    Returns:
        Matrice homographie 3x3
        
    Raises:
        ValueError: Si moins de 4 points
        
    Logs:
        [HOMOGRAPHY] Computed from N points
    """
    if len(src_points) < 4 or len(dst_points) < 4:
        raise ValueError("Au moins 4 points requis pour homographie")
    
    # Assurer type float32
    src = np.array(src_points, dtype=np.float32)
    dst = np.array(dst_points, dtype=np.float32)
    
    # Calculer homographie
    H, _ = cv2.findHomography(src, dst)
    
    print("[HOMOGRAPHY] Calculé depuis {} points".format(len(src_points)))
    
    return H


def apply_homography(points: np.ndarray, H: np.ndarray) -> np.ndarray:
    """
    Applique homographie à des points.
    
    Args:
        points: Points à transformer (Nx2)
        H: Matrice homographie 3x3
        
    Returns:
        Points transformés (Nx2)
    """
    # Convertir en coordonnées homogènes
    ones = np.ones((points.shape[0], 1))
    points_h = np.hstack([points, ones])
    
    # Appliquer transformation
    transformed_h = (H @ points_h.T).T
    
    # Normaliser (diviser par coordonnée w)
    transformed = transformed_h[:, :2] / transformed_h[:, 2:3]
    
    return transformed


def apply_homography_single(point: Tuple[float, float], 
                           H: np.ndarray) -> Tuple[float, float]:
    """
    Applique homographie à un point unique.
    
    Args:
        point: (x, y) point source
        H: Matrice homographie
        
    Returns:
        (x', y') point transformé
    """
    # Coordonnées homogènes
    p_h = np.array([point[0], point[1], 1.0])
    
    # Transformation
    p_transformed = H @ p_h
    
    # Normalisation
    x = p_transformed[0] / p_transformed[2]
    y = p_transformed[1] / p_transformed[2]
    
    return (x, y)


def create_scaling_matrix(scale: float) -> np.ndarray:
    """
    Crée matrice de scaling homogène.
    
    Args:
        scale: Facteur d'échelle
        
    Returns:
        Matrice 3x3
    """
    S = np.array([
        [scale, 0, 0],
        [0, scale, 0],
        [0, 0, 1]
    ], dtype=np.float32)
    
    return S


def combine_homographies(H1: np.ndarray, H2: np.ndarray) -> np.ndarray:
    """
    Combine deux homographies: H_combined = H2 @ H1.
    
    Args:
        H1: Première transformation
        H2: Deuxième transformation
        
    Returns:
        Homographie combinée
    """
    return H2 @ H1


def estimate_scale_from_marker(marker_corners_px: List[Tuple[float, float]],
                               H_C2AV: np.ndarray,
                               real_size_m: float) -> float:
    """
    Estime échelle métrique depuis marqueur ArUco.
    
    Args:
        marker_corners_px: 4 coins marqueur en pixels caméra
        H_C2AV: Homographie Caméra -> Arène Virtuelle
        real_size_m: Taille réelle marqueur en mètres
        
    Returns:
        Scale en mètres/unité AV
        
    Algorithm:
        1. Transformer coins en AV
        2. Calculer taille moyenne en AV
        3. scale = real_size_m / size_av
        
    Logs:
        [HOMOGRAPHY] Scale estimation: real=Xm, av=Y units -> scale=Z m/unit
    """
    # Transformer coins en AV
    corners_av = apply_homography(np.array(marker_corners_px), H_C2AV)
    
    # Calculer longueurs des 4 côtés
    side_lengths = []
    for i in range(4):
        j = (i + 1) % 4
        length = np.linalg.norm(corners_av[j] - corners_av[i])
        side_lengths.append(length)
    
    # Moyenne
    avg_size_av = np.mean(side_lengths)
    
    # Échelle
    scale = real_size_m / avg_size_av
    
    print("[HOMOGRAPHY] Estimation échelle : réel={:.3f}m, "
          "av={:.3f} unités -> échelle={:.3f} m/unité".format(real_size_m, avg_size_av, scale))
    
    return scale
