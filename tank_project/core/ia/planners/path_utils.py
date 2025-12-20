"""
Utilitaires de Chemin - Traitement et Optimisation de Chemin

Utilitaires pour travailler avec les chemins planifiés :
- Lissage de chemin
- Simplification de waypoints (Douglas-Peucker)
- Validation de chemin
- Calcul de distance
- Interpolation

Prend la sortie brute de A* et la rend prête à l'exécution.
"""

import numpy as np
from typing import List, Tuple, Optional


def smooth_path(waypoints: List[Tuple[float, float]], 
                weight_data: float = 0.5,
                weight_smooth: float = 0.3,
                tolerance: float = 0.01,
                max_iterations: int = 100) -> List[Tuple[float, float]]:
    """
    Lisse un chemin en utilisant la descente de gradient.
    
    Équilibre entre rester proche du chemin original vs lissage.
    
    Args:
        waypoints: Chemin original [(x1,y1), ...]
        weight_data: À quel point rester proche de l'original
        weight_smooth: À quel point lisser
        tolerance: Seuil de convergence
        max_iterations: Itérations maximum
        
    Returns:
        Chemin lissé
    """
    if len(waypoints) <= 2:
        return waypoints
    
    # Convertit en array numpy pour manipulation plus facile
    path = np.array(waypoints, dtype=np.float64)
    smoothed = path.copy()
    
    n_points = len(path)
    
    for iteration in range(max_iterations):
        change = 0.0
        
        # Ne modifie pas le premier et le dernier point
        for i in range(1, n_points - 1):
            for j in range(2):  # x et y
                old_val = smoothed[i, j]
                
                # Terme de données : rester proche de l'original
                data_term = weight_data * (path[i, j] - smoothed[i, j])
                
                # Terme de lissage : moyenne des voisins
                smooth_term = weight_smooth * (
                    smoothed[i-1, j] + smoothed[i+1, j] - 2 * smoothed[i, j]
                )
                
                smoothed[i, j] += data_term + smooth_term
                change += abs(smoothed[i, j] - old_val)
        
        # Vérifie la convergence
        if change < tolerance:
            break
    
    return [(float(p[0]), float(p[1])) for p in smoothed]


def simplify_path_douglas_peucker(waypoints: List[Tuple[float, float]], 
                                  epsilon: float = 0.05) -> List[Tuple[float, float]]:
    """
    Simplifie le chemin avec l'algorithme Douglas-Peucker.
    
    Supprime les waypoints qui sont presque colinéaires.
    
    Args:
        waypoints: Chemin original
        epsilon: Tolérance de déviation maximale en mètres
        
    Returns:
        Chemin simplifié avec moins de waypoints
        
    Algorithme :
        1. Trouve le point le plus éloigné de la ligne entre départ et fin
        2. Si distance < epsilon, supprime tous les points intermédiaires
        3. Sinon, applique récursivement à [départ, plus_lointain] et [plus_lointain, fin]
    """
    if len(waypoints) <= 2:
        return waypoints
    
    # Convertit en numpy pour les calculs
    points = np.array(waypoints)
    
    # Trouve le point avec la distance maximale de la ligne (départ -> fin)
    start = points[0]
    end = points[-1]
    
    # Vecteur ligne
    line_vec = end - start
    line_len = np.linalg.norm(line_vec)
    
    if line_len < 1e-10:
        # Départ et fin sont le même point
        return [waypoints[0], waypoints[-1]]
    
    line_unit = line_vec / line_len
    
    # Trouve les distances perpendiculaires
    max_dist = 0.0
    max_idx = 0
    
    for i in range(1, len(points) - 1):
        # Vecteur du départ au point
        vec_to_point = points[i] - start
        
        # Projette sur la ligne
        proj_length = np.dot(vec_to_point, line_unit)
        proj_point = start + proj_length * line_unit
        
        # Distance perpendiculaire
        dist = np.linalg.norm(points[i] - proj_point)
        
        if dist > max_dist:
            max_dist = dist
            max_idx = i
    
    # Si la distance max est inférieure à epsilon, simplifie aux extrémités
    if max_dist < epsilon:
        return [waypoints[0], waypoints[-1]]
    
    # Sinon, simplifie récursivement
    left_simplified = simplify_path_douglas_peucker(waypoints[:max_idx + 1], epsilon)
    right_simplified = simplify_path_douglas_peucker(waypoints[max_idx:], epsilon)
    
    # Combine (évite de dupliquer le point de coupure)
    return left_simplified[:-1] + right_simplified


def validate_path(waypoints: List[Tuple[float, float]], 
                 occupancy_grid) -> bool:
    """
    Vérifie si le chemin est sans collision.
    
    Args:
        waypoints: Chemin à valider
        occupancy_grid: Grille d'occupation actuelle
        
    Returns:
        True si le chemin est valide (pas de collisions)
    """
    if len(waypoints) < 2:
        return True
    
    # Vérifie chaque waypoint
    for x, y in waypoints:
        if not (0 <= x <= occupancy_grid.width_m and 0 <= y <= occupancy_grid.height_m):
            return False
        if occupancy_grid.is_occupied(x, y):
            return False
    
    # Vérifie les segments de ligne entre waypoints
    resolution = occupancy_grid.resolution
    
    for i in range(len(waypoints) - 1):
        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i + 1]
        
        # Echantillonne les points le long du segment
        dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        n_samples = max(2, int(dist / resolution) + 1)
        
        for t in np.linspace(0, 1, n_samples):
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            
            if occupancy_grid.is_occupied(x, y):
                return False
    
    return True


def calculate_path_length(waypoints: List[Tuple[float, float]]) -> float:
    """
    Calcule la longueur totale du chemin en mètres.
    
    Args:
        waypoints: Chemin [(x1,y1), ...]
        
    Returns:
        Longueur totale en mètres
    """
    if len(waypoints) < 2:
        return 0.0
    
    length = 0.0
    for i in range(len(waypoints) - 1):
        dx = waypoints[i+1][0] - waypoints[i][0]
        dy = waypoints[i+1][1] - waypoints[i][1]
        length += np.sqrt(dx**2 + dy**2)
    
    return length


def interpolate_path(waypoints: List[Tuple[float, float]], 
                    resolution: float = 0.05) -> List[Tuple[float, float]]:
    """
    Densifie le chemin en interpolant entre les waypoints.
    
    Utile pour une visualisation fluide ou un contrôle fin.
    
    Args:
        waypoints: Chemin clairsemé
        resolution: Espacement désiré en mètres
        
    Returns:
        Chemin dense avec des points tous les ~resolution mètres
    """
    if len(waypoints) < 2:
        return waypoints
    
    dense_path = [waypoints[0]]
    
    for i in range(len(waypoints) - 1):
        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i + 1]
        
        # Distance entre waypoints consécutifs
        dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        if dist < resolution:
            # Pas d'interpolation nécessaire
            dense_path.append((x2, y2))
            continue
        
        # Nombre de points intermédiaires
        n_points = int(dist / resolution)
        
        for j in range(1, n_points + 1):
            t = j / (n_points + 1)
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            dense_path.append((x, y))
        
        dense_path.append((x2, y2))
    
    return dense_path


def get_path_curvature(waypoints: List[Tuple[float, float]]) -> List[float]:
    """
    Calcule la courbure à chaque waypoint.
    
    Utile pour l'adaptation de vitesse (ralentir aux virages serrés).
    
    Args:
        waypoints: Chemin [(x1,y1), ...]
        
    Returns:
        Liste des valeurs de courbure (1/rayon, 0 pour ligne droite)
    """
    if len(waypoints) < 3:
        return [0.0] * len(waypoints)
    
    curvatures = [0.0]  # Le premier point n'a pas de courbure
    
    for i in range(1, len(waypoints) - 1):
        p0 = np.array(waypoints[i - 1])
        p1 = np.array(waypoints[i])
        p2 = np.array(waypoints[i + 1])
        
        # Vecteurs
        v1 = p1 - p0
        v2 = p2 - p1
        
        # Magnitude produit vectoriel (2D)
        cross = abs(v1[0] * v2[1] - v1[1] * v2[0])
        
        # Longueurs segments
        len1 = np.linalg.norm(v1)
        len2 = np.linalg.norm(v2)
        
        if len1 < 1e-10 or len2 < 1e-10:
            curvatures.append(0.0)
            continue
        
        # Approximation courbure : 2 * sin(angle) / longueur_corde
        # Simplifié : cross / (len1 * len2)
        curvature = cross / (len1 * len2)
        curvatures.append(curvature)
    
    curvatures.append(0.0)  # Le dernier point n'a pas de courbure
    
    return curvatures
