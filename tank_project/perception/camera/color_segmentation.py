"""
Color Segmentation - Détection Obstacles par Seuillage

Segmente les obstacles sur fond blanc par seuillage couleur:
- Détection zones sombres (obstacles)
- Masques binaires
- Filtrage bruit

Utilisé pendant la calibration pour cartographier obstacles statiques.

Logs: [SEGMENT] prefix
"""

import cv2
import numpy as np
from typing import Tuple


def threshold_obstacles(image: np.ndarray, 
                       threshold_value: int = 200) -> np.ndarray:
    """
    Seuillage simple pour détecter obstacles sur fond blanc.
    
    Args:
        image: Image BGR ou grayscale
        threshold_value: Seuil (pixels < threshold = obstacles)
        
    Returns:
        Masque binaire (0 = libre, 255 = obstacle)
    """
    # Convertir en niveaux de gris si nécessaire
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    
    # Seuillage inverse (obstacles sont sombres)
    _, binary = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY_INV)
    
    return binary


def adaptive_threshold_obstacles(image: np.ndarray) -> np.ndarray:
    """
    Seuillage adaptatif pour conditions éclairage variables.
    
    Args:
        image: Image BGR ou grayscale
        
    Returns:
        Masque binaire
    """
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    
    # Seuillage adaptatif
    binary = cv2.adaptiveThreshold(
        gray, 255, 
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        blockSize=11,
        C=2
    )
    
    return binary


def remove_noise(binary_mask: np.ndarray, 
                kernel_size: int = 5) -> np.ndarray:
    """
    Retire le bruit du masque binaire.
    
    Args:
        binary_mask: Masque binaire
        kernel_size: Taille kernel morphologie
        
    Returns:
        Masque filtré
    """
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    
    # Opening (erosion puis dilatation) pour retirer petits points
    opened = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel)
    
    # Closing (dilatation puis erosion) pour remplir trous
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
    
    return closed


def segment_obstacles(image: np.ndarray, 
                     method: str = 'simple',
                     denoise: bool = True) -> np.ndarray:
    """
    Pipeline complet de segmentation obstacles.
    
    Args:
        image: Image source
        method: 'simple' ou 'adaptive'
        denoise: Appliquer filtrage bruit
        
    Returns:
        Masque binaire nettoyé
        
    Logs:
        [SEGMENT] Obstacles detected: N pixels
    """
    if method == 'adaptive':
        mask = adaptive_threshold_obstacles(image)
    else:
        mask = threshold_obstacles(image)
    
    if denoise:
        mask = remove_noise(mask)
    
    # Compter pixels obstacles
    obstacle_pixels = np.count_nonzero(mask)
    print("[SEGMENT] Obstacles détectés : {} pixels".format(obstacle_pixels))
    
    return mask
