"""
Décisions - Fonctions de Décision Tactique

Fournit les fonctions d'évaluation tactique utilisées par les conditions de l'arbre comportemental :
- L'ennemi est-il trop proche ? (évaluation de la menace)
- A-t-on une ligne de vue ? (vérification visibilité)
- Sommes-nous à portée de tir optimale ?
- Y a-t-il une couverture à proximité ?
- Devrions-nous nous replier ?

Toutes les fonctions prennent un dict context et retournent bool ou une valeur tactique.

Logs : [DECISION] Évaluation X : valeur Y
"""

import numpy as np
from typing import Dict, Tuple, Optional, List


def is_enemy_too_close(context: Dict, threshold_m: float = 0.8) -> bool:
    """
    Vérifie si l'ennemi est dangereusement proche.
    
    Args:
        context: État du monde avec poses robots
        threshold_m: Seuil de danger en mètres
        
    Returns:
        True si ennemi dans le seuil
    """
    ai_pos = context['ai_pose'][:2]
    human_pos = context['human_pose'][:2]
    distance = np.linalg.norm(np.array(ai_pos) - np.array(human_pos))
    
    return distance < threshold_m


def has_line_of_sight(context: Dict) -> bool:
    """
    Vérifie si l'IA a une ligne de vue dégagée vers l'ennemi.
    
    Utilise le raycast de core.world pour vérifier les obstacles.
    
    Args:
        context: État du monde avec poses robots et raycast
        
    Returns:
        True si une ligne de vue claire existe
        
    Logs :
        [DECISION] Verif LOS : CLAIR/BLOQUE
    """
    ai_pose = context.get('ai_pose')
    human_pose = context.get('human_pose')
    raycast = context.get('raycast_sys')
    
    if ai_pose is None or human_pose is None:
        return False
    
    if raycast is None:
        # Pas de système raycast disponible, suppose LOS dégagée
        print("[DECISION] Verif LOS : PAS DE SYSTEME RAYCAST")
        return True
    
    # Utilise la vérification LOS interne du raycast
    ai_pos = ai_pose[:2]
    human_pos = human_pose[:2]
    
    los_clear = raycast._check_line_of_sight(ai_pos, human_pos)
    
    status = "CLAIR" if los_clear else "BLOQUE"
    print("[DECISION] Verif LOS : {}".format(status))
    
    return los_clear


def is_optimal_firing_range(context: Dict, 
                            min_range: float = 1.2, 
                            max_range: float = 3.5) -> bool:
    """
    Vérifie si l'ennemi est à portée de tir optimale.
    
    Trop proche : risque de riposte
    Trop loin : la précision diminue
    
    Args:
        context: État du monde
        min_range: Distance minimale de sécurité
        max_range: Distance effective maximale
        
    Returns:
        True si dans la portée optimale
    """
    ai_pose = context.get('ai_pose')
    human_pose = context.get('human_pose')
    
    if ai_pose is None or human_pose is None:
        return False
    
    ai_pos = np.array(ai_pose[:2])
    human_pos = np.array(human_pose[:2])
    
    distance = np.linalg.norm(ai_pos - human_pos)
    
    in_range = min_range <= distance <= max_range
    
    print("[DECISION] Verif portée tir : distance={:.2f}m, optimal={}".format(
        distance, in_range))
    
    return in_range


def find_nearest_cover(context: Dict) -> Optional[Tuple[float, float]]:
    """
    Trouve la position de couverture la plus proche par rapport à l'ennemi.
    
    Couverture = obstacle qui bloque la ligne de vue vers l'ennemi.
    
    Args:
        context: État du monde avec grille d'occupation
        
    Returns:
        (x, y) position de la meilleure couverture, ou None
        
    Algorithme :
        1. Récupère toutes les cellules d'obstacle de la grille
        2. Pour chaque obstacle, vérifie s'il bloque la LOS vers l'ennemi
        3. Classe par :
           - Distance à l'IA (plus proche est mieux)
           - Efficacité de la couverture (bloque bien la LOS)
        4. Retourne la meilleure position
    """
    ai_pose = context.get('ai_pose')
    human_pose = context.get('human_pose')
    grid = context.get('occupancy_grid')
    
    if ai_pose is None or human_pose is None or grid is None:
        return None
    
    ai_pos = np.array(ai_pose[:2])
    human_pos = np.array(human_pose[:2])
    
    # Direction de l'ennemi vers l'IA
    direction = ai_pos - human_pos
    dist = np.linalg.norm(direction)
    if dist < 0.1:
        return None
    direction = direction / dist
    
    # Cherche des positions de couverture : déplacement perpendiculaire à la direction ennemie
    perpendicular = np.array([-direction[1], direction[0]])
    
    # Vérifie les positions à gauche et à droite de la position actuelle
    cover_distance = 0.5  # mètres de la position actuelle
    
    candidates = [
        ai_pos + perpendicular * cover_distance,
        ai_pos - perpendicular * cover_distance,
        ai_pos + direction * cover_distance,  # S'éloigne de l'ennemi
    ]
    
    # Trouve une position de couverture valide (dans les limites de l'arène)
    for candidate in candidates:
        x, y = candidate
        if 0 < x < grid.width_m and 0 < y < grid.height_m:
            if not grid.is_occupied(x, y):
                print("[DECISION] Couverture trouvée à ({:.2f}, {:.2f})".format(x, y))
                return (x, y)
    
    print("[DECISION] Aucune couverture trouvée")
    return None


def should_retreat(context: Dict) -> bool:
    """
    Décision de repli complète.
    
    Repli si :
    - Ennemi trop proche ET a la LOS
    - Santé faible (fonctionnalité future)
    - Encerclé
    
    Args:
        context: État du monde
        
    Returns:
        True si devrait se replier
    """
    too_close = is_enemy_too_close(context)
    los = has_line_of_sight(context)
    
    should_run = too_close and los
    
    if should_run:
        print("[DECISION] REPLI déclenché : ennemi trop proche avec LOS")
    
    return should_run


def calculate_flank_position(context: Dict) -> Optional[Tuple[float, float]]:
    """
    Calcule la position de contournement optimale.
    
    But : position qui :
    - Donne à l'IA une ligne de vue sur l'ennemi
    - N'est PAS dans la ligne de vue actuelle de l'ennemi
    - Utilise une couverture pour l'approche
    
    Args:
        context: État du monde
        
    Returns:
        (x, y) position cible de contournement
        
    Algorithme :
        1. Récupère position et orientation ennemi
        2. Trouve positions 90 deg gauche/droite de l'orientation ennemie
        3. Filtre par disponibilité couverture pendant l'approche
        4. Choisit la position valide la plus proche
    """
    ai_pose = context.get('ai_pose')
    human_pose = context.get('human_pose')
    grid = context.get('occupancy_grid')
    
    if ai_pose is None or human_pose is None:
        return None
    
    ai_pos = np.array(ai_pose[:2])
    human_pos = np.array(human_pose[:2])
    human_theta = human_pose[2] if len(human_pose) > 2 else 0.0
    
    # Direction du regard ennemi
    enemy_facing = np.array([np.cos(human_theta), np.sin(human_theta)])
    
    # Positions de flanc : 90 degrés par rapport au regard ennemi
    flank_distance = 1.5  # mètres de l'ennemi
    
    # Flanc gauche (perpendiculaire)
    left_perp = np.array([-enemy_facing[1], enemy_facing[0]])
    left_flank = human_pos + left_perp * flank_distance
    
    # Flanc droit
    right_perp = np.array([enemy_facing[1], -enemy_facing[0]])
    right_flank = human_pos + right_perp * flank_distance
    
    # Choisit le flanc le plus proche de l'IA
    dist_left = np.linalg.norm(ai_pos - left_flank)
    dist_right = np.linalg.norm(ai_pos - right_flank)
    
    if dist_left < dist_right:
        chosen_flank = left_flank
    else:
        chosen_flank = right_flank
    
    x, y = chosen_flank
    
    # Valide que la position est dans l'arène
    if grid is not None:
        if not (0 < x < grid.width_m and 0 < y < grid.height_m):
            print("[DECISION] Position contournement hors limites")
            return None
        if grid.is_occupied(x, y):
            print("[DECISION] Position contournement occupée")
            return None
    
    print("[DECISION] Position contournement : ({:.2f}, {:.2f})".format(x, y))
    return (x, y)


def get_distance_to_enemy(context: Dict) -> float:
    """
    Obtient la distance entre l'IA et l'ennemi.
    
    Args:
        context: État du monde
        
    Returns:
        Distance en mètres, ou l'infini si poses inconnues
    """
    ai_pose = context.get('ai_pose')
    human_pose = context.get('human_pose')
    
    if ai_pose is None or human_pose is None:
        return float('inf')
    
    ai_pos = np.array(ai_pose[:2])
    human_pos = np.array(human_pose[:2])
    
    return float(np.linalg.norm(ai_pos - human_pos))


def get_angle_to_enemy(context: Dict) -> float:
    """
    Obtient l'angle de l'IA vers l'ennemi.
    
    Args:
        context: État du monde
        
    Returns:
        Angle en radians, ou 0 si poses inconnues
    """
    ai_pose = context.get('ai_pose')
    human_pose = context.get('human_pose')
    
    if ai_pose is None or human_pose is None:
        return 0.0
    
    dx = human_pose[0] - ai_pose[0]
    dy = human_pose[1] - ai_pose[1]
    
    return float(np.arctan2(dy, dx))
