"""
Contours - Extraction Contours

Extraction et traitement des contours d'image.

Logs: [CONTOURS] prefix
"""

import cv2
import numpy as np
from typing import List, Tuple


def find_contours(binary_image: np.ndarray) -> List[np.ndarray]:
    """
    Trouve contours dans image binaire.
    
    Args:
        binary_image: Image binaire
        
    Returns:
        Liste de contours
    """
    contours, _ = cv2.findContours(
        binary_image,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )
    
    print("[CONTOURS] Trouvé {} contours".format(len(contours)))
    
    return contours


def filter_contours_by_area(contours: List[np.ndarray],
                           min_area: float = 100.0,
                           max_area: float = np.inf) -> List[np.ndarray]:
    """
    Filtre contours par aire.
    
    Args:
        contours: Liste contours
        min_area: Aire minimum
        max_area: Aire maximum
        
    Returns:
        Contours filtrés
    """
    filtered = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if min_area <= area <= max_area:
            filtered.append(cnt)
    
    print("[CONTOURS] Filtré {} -> {} contours".format(len(contours), len(filtered)))
    
    return filtered


def get_bounding_boxes(contours: List[np.ndarray]) -> List[Tuple[int, int, int, int]]:
    """
    Extrait rectangles englobants.
    
    Args:
        contours: Liste contours
        
    Returns:
        Liste (x, y, w, h) rectangles
    """
    boxes = []
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        boxes.append((x, y, w, h))
    
    return boxes


def approximate_polygons(contours: List[np.ndarray],
                        epsilon_factor: float = 0.02) -> List[np.ndarray]:
    """
    Approxime contours par polygones.
    
    Args:
        contours: Liste contours
        epsilon_factor: Facteur précision (% périmètre)
        
    Returns:
        Contours approximés
    """
    approximated = []
    for cnt in contours:
        epsilon = epsilon_factor * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        approximated.append(approx)
    
    return approximated
