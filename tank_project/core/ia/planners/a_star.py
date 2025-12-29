"""
Algorithme de Planification A*

Implémente la recherche de chemin A* sur la grille d'occupation :
- Trouve le chemin le plus court sans collision
- Utilise des heuristiques configurables
- Gère les obstacles dynamiques (robots)
- Retourne une liste de waypoints en mètres

Le planificateur travaille sur la carte de coût (costmap) gonflée de core/world.

Logs : [ASTAR] Chemin trouvé : N waypoints, longueur : M mètres
"""

import numpy as np
import heapq
from typing import List, Tuple, Optional
from .heuristics import euclidean_distance, manhattan_distance, diagonal_distance


class AStarPlanner:
    """
    Recherche de chemin A* sur grille d'occupation 2D.
    
    Trouve le chemin optimal du départ au but en évitant les obstacles.
    """
    
    def __init__(self, occupancy_grid, heuristic='euclidean'):
        """
        Initialise le planificateur A*.
        
        Args:
            occupancy_grid: OccupancyGrid depuis core/world
            heuristic: 'euclidean', 'manhattan', ou 'diagonal'
        """
        self.grid = occupancy_grid
        self.heuristic_name = heuristic
        
    def _heuristic(self, cell1, cell2):
        """Calcule le coût heuristique."""
        if self.heuristic_name == 'manhattan':
            return manhattan_distance(cell1, cell2)
        elif self.heuristic_name == 'diagonal':
            return diagonal_distance(cell1, cell2)
        else:
            return euclidean_distance(cell1, cell2)

    
    def plan(self, start_m: Tuple[float, float], 
             goal_m: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        Trouve un chemin du départ au but.
        """
        start_cell = self.grid.world_to_grid(*start_m)
        goal_cell = self.grid.world_to_grid(*goal_m)
        
        print(f"[ASTAR] Planification de {start_m} vers {goal_m}")
        print(f"[ASTAR]   Cellule départ : {start_cell}, Cellule but : {goal_cell}")
        print(f"[ASTAR]   Taille grille : {self.grid.grid.shape}")
        
        # Vérification des limites
        if not self.grid._is_valid_cell(*start_cell):
            print(f"[ASTAR]   ERREUR : Cellule départ {start_cell} HORS LIMITES")
            return None
        if not self.grid._is_valid_cell(*goal_cell):
            print(f"[ASTAR]   ERREUR : Cellule but {goal_cell} HORS LIMITES")
            return None
        
        # Note : On ne vérifie pas si le départ ou le but est occupé
        # Départ : le robot est déjà là
        # But : le robot ennemi est là - on VEUT aller là-bas !
            
        # Initialisation
        open_set = []
        heapq.heappush(open_set, (0, start_cell))
        
        came_from = {}
        g_score = {start_cell: 0}
        f_score = {start_cell: self._heuristic(start_cell, goal_cell)}
        
        closed_set = set()
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current == goal_cell:
                path_cells = self._reconstruct_path(came_from, current)
                simplified = self._simplify_path(path_cells)
                result = [self.grid.grid_to_world(r, c) for r, c in simplified]
                print(f"[ASTAR]   SUCCES : Chemin trouvé avec {len(result)} waypoints")
                return result
            
            closed_set.add(current)
            
            for neighbor, cost in self._get_neighbors(current, start_cell):
                if neighbor in closed_set:
                    continue
                    
                tentative_g = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self._heuristic(neighbor, goal_cell)
                    f_score[neighbor] = f
                    heapq.heappush(open_set, (f, neighbor))
        
        print(f"[ASTAR]   ECHEC : Aucun chemin trouvé (exploré {len(closed_set)} cellules)")
        return None
        
    def _get_neighbors(self, cell, start_cell=None, ignore_radius=5):
        """Obtient les cellules voisines valides (8-connexité).
        
        Utilise la COSTMAP (gonflée) pour la vérification de collision, pas la grille brute.
        Si start_cell est fourni et qu'on est dans ignore_radius,
        on ignore les obstacles (pour s'échapper de l'empreinte du robot).
        """
        row, col = cell
        neighbors = []
        
        # Utilise la costmap si disponible (obstacles gonflés), sinon grille
        collision_grid = getattr(self.grid, 'costmap', self.grid.grid)
        
        for dr in [-1, 0, 1]:
            for dc in [-1, 0, 1]:
                if dr == 0 and dc == 0:
                    continue
                    
                r, c = row + dr, col + dc
                
                # Vérifie la validité
                if self.grid._is_valid_cell(r, c):
                    # Si près du départ, ignore les obstacles (empreinte du robot)
                    if start_cell is not None:
                        dist_to_start = abs(r - start_cell[0]) + abs(c - start_cell[1])
                        if dist_to_start <= ignore_radius:
                            cost = 1.414 if (dr != 0 and dc != 0) else 1.0
                            neighbors.append(((r, c), cost))
                            continue
                    
                    # Vérifie l'occupation sur la COSTMAP (gonflée, marges de sécurité)
                    if collision_grid[r, c] < 0.5:
                        cost = 1.414 if (dr != 0 and dc != 0) else 1.0
                        neighbors.append(((r, c), cost))
                        
        return neighbors
    
    def _reconstruct_path(self, came_from, current):
        """Reconstruit le chemin du but au départ."""
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1] # Inverse
    
    def _simplify_path(self, path_cells, epsilon=1.5):
        """
        Simplification du chemin via algorithme Douglas-Peucker.
        
        Args:
            path_cells: Liste de cellules (row, col)
            epsilon: Tolérance de simplification (en cellules)
            
        Returns:
            Liste simplifiée de cellules
        """
        if len(path_cells) <= 2:
            return path_cells
        
        # Trouve le point le plus éloigné de la ligne start-end
        start = np.array(path_cells[0])
        end = np.array(path_cells[-1])
        
        max_dist = 0
        max_idx = 0
        
        line_vec = end - start
        line_len = np.linalg.norm(line_vec)
        
        if line_len == 0:
            return [path_cells[0], path_cells[-1]]
        
        line_unit = line_vec / line_len
        
        for i in range(1, len(path_cells) - 1):
            point = np.array(path_cells[i])
            vec_to_point = point - start
            
            # Distance perpendiculaire à la ligne
            proj_length = np.dot(vec_to_point, line_unit)
            proj_point = start + proj_length * line_unit
            dist = np.linalg.norm(point - proj_point)
            
            if dist > max_dist:
                max_dist = dist
                max_idx = i
        
        # Si le point le plus éloigné dépasse epsilon, récursion
        if max_dist > epsilon:
            left = self._simplify_path(path_cells[:max_idx + 1], epsilon)
            right = self._simplify_path(path_cells[max_idx:], epsilon)
            return left[:-1] + right
        else:
            return [path_cells[0], path_cells[-1]]
