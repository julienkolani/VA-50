"""
Modèle du Monde - Représentation Unifiée du Monde

Référentiel central pour tout l'état du monde :
- Poses robots (filtrées par Kalman)
- Grille d'occupation (obstacles)
- Limites de l'arène
- Trames de coordonnées

C'est la source unique de vérité pour l'information spatiale.
Tous les autres modules interrogent le modèle du monde.

NE contient PAS la logique de jeu (scores, etc.) - seulement l'état spatial.
"""

from typing import Dict, List, Tuple
from .occupancy_grid import OccupancyGrid
from .coordinate_frames import TransformManager


class WorldModel:
    """
    Représentation spatiale complète du monde.
    
    Gère :
    - État des robots (positions, vitesses, orientations)
    - Obstacles (statiques + dynamiques)
    - Transformations de coordonnées
    - Limites de l'arène
    """
    
    def __init__(self, arena_width_m: float, arena_height_m: float, 
                 grid_resolution_m: float = 0.02,
                 robot_radius_m: float = 0.09,
                 inflation_margin_m: float = 0.05):
        """
        Initialise le modèle du monde.
        
        Args:
            arena_width_m: Largeur de l'arène depuis l'étalonnage
            arena_height_m: Hauteur de l'arène depuis l'étalonnage
            grid_resolution_m: Taille de cellule grille
            robot_radius_m: Rayon physique du robot (depuis config)
            inflation_margin_m: Marge de sécurité pour pathfinding (depuis config)
        """
        self.arena_width = arena_width_m
        self.arena_height = arena_height_m
        self.robot_radius_m = robot_radius_m
        self.inflation_margin_m = inflation_margin_m
        
        # Grille d'occupation
        self.grid = OccupancyGrid(arena_width_m, arena_height_m, grid_resolution_m)
        
        # État du robot
        self.robots = {
            4: {  # Robot IA
                'pose': (0.0, 0.0, 0.0),  # (x, y, theta)
                'velocity': (0.0, 0.0, 0.0),  # (vx, vy, omega)
                'radius_m': robot_radius_m,
            },
            5: {  # Robot Humain
                'pose': (0.0, 0.0, 0.0),
                'velocity': (0.0, 0.0, 0.0),
                'radius_m': robot_radius_m,
            }
        }
        
        # Transformations de coordonnées
        self.transforms = TransformManager()
        
    def update_robot_pose(self, robot_id: int, pose: Tuple[float, float, float]):
        """
        Met à jour la pose du robot depuis le filtre de Kalman.
        
        Args:
            robot_id: 4 ou 5
            pose: (x, y, theta) en mètres/radians
        """
        if robot_id in self.robots:
            self.robots[robot_id]['pose'] = pose
    
    def update_robot_velocity(self, robot_id: int, 
                             velocity: Tuple[float, float, float]):
        """
        Met à jour la vitesse du robot depuis le filtre de Kalman.
        
        Args:
            robot_id: 4 ou 5
            velocity: (vx, vy, omega) en m/s et rad/s
        """
        if robot_id in self.robots:
            self.robots[robot_id]['velocity'] = velocity
    
    def update_occupancy(self):
        """
        Met à jour la grille d'occupation avec les positions actuelles des robots.
        
        Appelé chaque frame après la mise à jour des poses des robots.
        """
        robot_poses = [self.robots[rid]['pose'] for rid in [4, 5]]
        self.grid.update_dynamic_obstacles(robot_poses, self.robot_radius_m)
    
    def generate_costmap(self):
        """
        Génère la costmap gonflée pour le pathfinding A*.
        
        Appelle après avoir chargé les obstacles statiques.
        Utilise les paramètres robot du config.
        """
        self.grid.inflate_static_obstacles(self.robot_radius_m, self.inflation_margin_m)
        print(f"[WORLD] Costmap générée avec rayon={self.robot_radius_m}m, marge={self.inflation_margin_m}m")
    
    def get_robot_pose(self, robot_id: int) -> Tuple[float, float, float]:
        """Obtient la pose actuelle du robot."""
        return self.robots[robot_id]['pose']
    
    def get_robot_velocity(self, robot_id: int) -> Tuple[float, float, float]:
        """Obtient la vitesse actuelle du robot."""
        return self.robots[robot_id]['velocity']
    
    def is_position_valid(self, x: float, y: float) -> bool:
        """
        Vérifie si une position est dans l'arène et non occupée.
        
        Args:
            x, y: Position en mètres
            
        Returns:
            True si la position est valide (dans les limites et libre)
        """
        # Vérifie les limites
        if not (0 <= x <= self.arena_width and 0 <= y <= self.arena_height):
            return False
        
        # Vérifie l'occupation
        return not self.grid.is_occupied(x, y)
    
    def get_state_dict(self) -> Dict:
        """
        Exporte l'état complet du monde sous forme de dictionnaire.
        
        Utilisé par l'IA, le moteur de jeu, la visualisation.
        
        Returns:
            dict avec toutes les informations du monde
        """
        return {
            'arena_size': (self.arena_width, self.arena_height),
            'robot_4_pose': self.robots[4]['pose'],
            'robot_5_pose': self.robots[5]['pose'],
            'robot_4_velocity': self.robots[4]['velocity'],
            'robot_5_velocity': self.robots[5]['velocity'],
            'occupancy_grid': self.grid,
        }
