"""
Scale Estimator - Estimation Échelle Métrique

Estime le facteur d'échelle de AV -> Monde en mètres
à partir d'un marqueur ArUco physique de taille connue.

Logs: [SCALE_EST] prefix
"""

import numpy as np
from typing import List, Tuple


class ScaleEstimator:
    """
    Estime l'échelle métrique depuis marqueur physique.
    """
    
    def __init__(self, marker_real_size_m: float = 0.10):
        """
        Initialize scale estimator.
        
        Args:
            marker_real_size_m: Taille réelle marqueur en mètres
        """
        self.marker_size_real = marker_real_size_m
        self.samples = []
        
    def estimate_from_corners(self,
                            corners_av: np.ndarray) -> float:
        """
        Estime échelle depuis coins marqueur en coordonnées AV.
        
        Args:
            corners_av: 4 coins en unités AV (4x2)
            
        Returns:
            Échelle en m/unité_av
        """
        # Calculer longueur moyenne des côtés en AV
        side_lengths = []
        for i in range(4):
            j = (i + 1) % 4
            length = np.linalg.norm(corners_av[j] - corners_av[i])
            side_lengths.append(length)
        
        avg_size_av = np.mean(side_lengths)
        
        # Échelle
        scale = self.marker_size_real / avg_size_av
        
        return scale
    
    def add_sample(self, corners_av: np.ndarray):
        """
        Ajoute une mesure d'échelle.
        
        Args:
            corners_av: Coins marqueur en AV
        """
        scale = self.estimate_from_corners(corners_av)
        self.samples.append(scale)
        print("[SCALE_EST] Échantillon {} : échelle={:.4f} m/unité".format(len(self.samples), scale))
        
    def get_average_scale(self) -> float:
        """
        Retourne échelle moyenne de tous les échantillons.
        
        Returns:
            Échelle moyenne
            
        Logs:
            [SCALE_EST] Average scale from N samples: X m/unit (std=Y)
        """
        if not self.samples:
            raise ValueError("Aucun échantillon disponible")
        
        avg = np.mean(self.samples)
        std = np.std(self.samples)
        
        print("[SCALE_EST] Échelle moyenne sur {} échantillons : "
              "{:.4f} m/unité (std={:.4f})".format(len(self.samples), avg, std))
        
        return avg
    
    def reset(self):
        """Réinitialise les échantillons."""
        self.samples = []
