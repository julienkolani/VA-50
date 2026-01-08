"""
Grille d'Occupation - Représentation Spatiale 2D

Représente l'arène comme une grille 2D avec résolution métrique :
- Valeurs cellules : 0 = libre, 1 = occupé, 0-1 = partiel
- Résolution : typiquement 2cm (0.02m) par cellule
- Dimensions : dérivées de l'étalonnage

La grille stocke :
- Obstacles statiques (depuis l'étalonnage)
- Obstacles dynamiques (robots, mis à jour chaque frame)
- Obstacles gonflés (marges de sécurité)

Système de coordonnées : mètres, origine en bas à gauche de l'arène.

Logs : préfixe [GRID] pour toutes les opérations de grille
"""

import numpy as np
from typing import Tuple, List


class OccupancyGrid:
    """
    Grille d'occupation 2D pour représentation spatiale.
    
    Fournit une vérification de collision efficace et des requêtes spatiales.
    """
    
    def __init__(self, width_m: float, height_m: float, resolution_m: float = 0.02):
        """
        Initialise la grille d'occupation.
        
        Args:
            width_m: Largeur de l'arène en mètres
            height_m: Hauteur de l'arène en mètres  
            resolution_m: Taille de cellule en mètres (défaut 2cm)
            
        La grille aura les dimensions :
            n_cols = ceil(width_m / resolution_m)
            n_rows = ceil(height_m / resolution_m)
            
        Logs:
            [GRID] Grille créée : 2.85m x 1.90m @ 0.02m -> 143 x 95 cellules
        """
        self.width_m = width_m
        self.height_m = height_m
        self.resolution = resolution_m
        
        self.n_cols = int(np.ceil(width_m / resolution_m))
        self.n_rows = int(np.ceil(height_m / resolution_m))
        
        # Données grille : 0 = libre, 1 = occupé
        self.grid = np.zeros((self.n_rows, self.n_cols), dtype=np.float32)
        
        # Obstacles statiques (depuis étalonnage, ne changent jamais)
        self.static_grid = np.zeros((self.n_rows, self.n_cols), dtype=np.float32)
        
    def world_to_grid(self, x_m: float, y_m: float) -> Tuple[int, int]:
        """
        Convertit coordonnées monde en cellule grille.
        
        Args:
            x_m: Position X en mètres
            y_m: Position Y en mètres
            
        Returns:
            (lig, col) indices cellule grille
        """
        col = int(x_m / self.resolution)
        row = int(y_m / self.resolution)
        return (row, col)
    
    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """
        Convertit cellule grille en coordonnées monde (centre cellule).
        
        Args:
            row: Ligne grille
            col: Colonne grille
            
        Returns:
            (x_m, y_m) en mètres
        """
        x_m = (col + 0.5) * self.resolution
        y_m = (row + 0.5) * self.resolution
        return (x_m, y_m)
    
    def is_occupied(self, x_m: float, y_m: float, threshold: float = 0.5) -> bool:
        """
        Vérifie si une position monde est occupée.
        
        Args:
            x_m: Position X en mètres
            y_m: Position Y en mètres
            threshold: Seuil d'occupation (0-1)
            
        Returns:
            True si occupée
        """
        row, col = self.world_to_grid(x_m, y_m)
        
        if not self._is_valid_cell(row, col):
            return True  # Hors limites = occupé
        
        return self.grid[row, col] >= threshold
    
    def get_value(self, x_m: float, y_m: float) -> float:
        """
        Obtient la valeur d'occupation à une position monde.
        
        Args:
            x_m: Position X en mètres
            y_m: Position Y en mètres
            
        Returns:
            Valeur d'occupation 0-100 (0=libre, 100=occupé)
        """
        row, col = self.world_to_grid(x_m, y_m)
        
        if not self._is_valid_cell(row, col):
            return 100  # Hors limites = occupé
        
        return self.grid[row, col] * 100
    
    def set_static_obstacles(self, obstacle_cells: List[Tuple[int, int]]):
        """
        Définit les obstacles statiques depuis l'étalonnage.
        
        Args:
            obstacle_cells: Liste des cellules occupées (lig, col)
            
        Logs:
            [GRID] Obstacles statiques définis : N cellules
        """
        for row, col in obstacle_cells:
            if self._is_valid_cell(row, col):
                self.static_grid[row, col] = 1.0
        
        # Par défaut, inflated_grid = static_grid (pas d'inflation)
        self.inflated_grid = self.static_grid.copy()
        self.grid = self.inflated_grid.copy()
        
        print(f"[GRID] Obstacles statiques définis : {len(obstacle_cells)} cellules")
    
    def inflate_static_obstacles(self, robot_radius_m: float, safety_margin_m: float = 0.0):
        """
        Génère une Costmap en gonflant les obstacles avec cv2.dilate (RAPIDE).
        
        Utilise OpenCV pour un calcul instantané même sur Raspberry Pi.
        Crée une zone tampon autour de chaque obstacle basée sur la taille physique du robot.
        
        Args:
            robot_radius_m: Rayon physique du robot (ex: 0.09m pour Turtlebot)
            safety_margin_m: Marge de sécurité supplémentaire (ex: 0.05m)
            
        Logs:
            [GRID] Inflation calculée: rayon + marge = total => kernel size
        """
        import cv2
        
        # 1. Calcul du rayon total à gonfler
        total_inflation_m = robot_radius_m + safety_margin_m
        
        # 2. Conversion Mètres -> Cellules (dynamique selon résolution)
        radius_cells = int(np.ceil(total_inflation_m / self.resolution))
        
        # 3. Calcul du Kernel pour OpenCV qui doit être impair
        kernel_size = (radius_cells * 2) + 1
        
        print(f"[GRID] l'Inflation calculée :")
        print(f"       Robot : {robot_radius_m}m + Marge : {safety_margin_m}m = {total_inflation_m}m")
        print(f"       Cellules : {total_inflation_m:.3f}m / {self.resolution}m = {radius_cells} cellules")
        print(f"       Noyau OpenCV : {kernel_size}x{kernel_size}")
        
        # 4. Création du Kernel Circulaire (forme du robot)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        
        # 5. Application de la dilatation (ULTRA RAPIDE vs boucles Python)
        static_u8 = (self.static_grid * 255).astype(np.uint8)
        inflated_u8 = cv2.dilate(static_u8, kernel)
        
        # 6. Sauvegarde dans costmap (pour A*) et inflated_grid
        self.costmap = (inflated_u8 > 127).astype(np.float32)
        self.inflated_grid = self.costmap.copy()
        
        # Appliquer à la grille courante
        self.grid = self.inflated_grid.copy()
    
    def update_dynamic_obstacles(self, robot_poses: List[Tuple[float, float, float]], 
                                 robot_radius_m: float):
        """
        Met à jour la grille avec les positions actuelles des robots.
        
        Args:
            robot_poses: Liste de (x, y, theta) pour chaque robot
            robot_radius_m: Rayon du robot en mètres
            
        Algorithm:
            1. Réinitialise la grille aux obstacles statiques gonflés
            2. Pour chaque robot, marque les cellules dans le rayon comme occupées
        """
        # Reset to inflated static (pas static_grid brut)
        self.grid = self.inflated_grid.copy() if hasattr(self, 'inflated_grid') else self.static_grid.copy()
        
        # Ajoute les empreintes des robots
        radius_cells = int(np.ceil(robot_radius_m / self.resolution))
        
        for x, y, _ in robot_poses:
            center_row, center_col = self.world_to_grid(x, y)
            
            # Marque le cercle de cellules
            for dr in range(-radius_cells, radius_cells + 1):
                for dc in range(-radius_cells, radius_cells + 1):
                    if dr**2 + dc**2 <= radius_cells**2:
                        r, c = center_row + dr, center_col + dc
                        if self._is_valid_cell(r, c):
                            self.grid[r, c] = 1.0
    
    def _is_valid_cell(self, row: int, col: int) -> bool:
        """Vérifie si la cellule est dans les limites de la grille."""
        return 0 <= row < self.n_rows and 0 <= col < self.n_cols
    
    def get_costmap(self):
        """
        Retourne la costmap actuelle pour la planification.
        
        Returns:
            numpy array (n_rows x n_cols) avec coûts 0-100
        """
        return (self.grid * 100).astype(np.uint8)

