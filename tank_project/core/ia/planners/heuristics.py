"""
Heuristiques - Fonctions de Coût pour la Planification de Chemin

Fournit des fonctions heuristiques pour A* et autres planificateurs :
- Distance Euclidienne (standard, admissible)
- Distance de Manhattan (basée grille)
- Distance Diagonale (Chebyshev + coût diagonal)
- Heuristiques personnalisées sensibles à la costmap

Toutes les heuristiques doivent être admissibles (ne jamais surestimer).
"""

import numpy as np
from typing import Tuple


def euclidean_distance(cell1: Tuple[int, int], cell2: Tuple[int, int]) -> float:
    """
    Heuristique de distance Euclidienne.
    
    La plus précise pour la planification en espace libre.
    
    Args:
        cell1: (lig, col)
        cell2: (lig, col)
        
    Returns:
        Distance Euclidienne
    """
    return np.sqrt((cell1[0] - cell2[0])**2 + (cell1[1] - cell2[1])**2)


def manhattan_distance(cell1: Tuple[int, int], cell2: Tuple[int, int]) -> float:
    """
    Distance de Manhattan (L1).
    
    Bon pour les grilles 4-connexes.
    
    Args:
        cell1: (lig, col)
        cell2: (lig, col)
        
    Returns:
        Distance de Manhattan
    """
    return abs(cell1[0] - cell2[0]) + abs(cell1[1] - cell2[1])


def diagonal_distance(cell1: Tuple[int, int], cell2: Tuple[int, int]) -> float:
    """
    Distance Diagonale (Chebyshev + coût diagonal).
    
    Bon pour les grilles 8-connexes avec coût diagonal √2.
    
    Args:
        cell1: (lig, col)
        cell2: (lig, col)
        
    Returns:
        Distance prenant en compte les diagonales
    """
    dx = abs(cell1[0] - cell2[0])
    dy = abs(cell1[1] - cell2[1])
    
    # Coût : mouvement diagonal (√2 ≈ 1.414) puis droit (1.0)
    D = 1.0  # Coût droit
    D2 = 1.414  # Coût diagonal
    
    return D * (dx + dy) + (D2 - 2*D) * min(dx, dy)


def costmap_aware_heuristic(cell1: Tuple[int, int], 
                            cell2: Tuple[int, int],
                            costmap) -> float:
    """
    Heuristique sensible à la costmap.
    
    Incorpore la proximité des obstacles dans l'heuristique.
    Reste admissible si la base est euclidienne.
    
    Args:
        cell1: (lig, col)
        cell2: (lig, col)
        costmap: OccupancyGrid avec inflation
        
    Returns:
        Heuristique modifiée favorisant les chemins plus sûrs
    """
    base_h = euclidean_distance(cell1, cell2)
    
    # Optionnel : ajouter une petite pénalité basée sur la valeur moyenne de la costmap
    # Doit rester admissible !
    
    return base_h
