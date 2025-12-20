"""
Filtre de Kalman - Suivi de Pose Robot

Filtre de Kalman Étendu (EKF) pour l'estimation d'état du robot :
- État : [x, y, vx, vy, theta, omega]
- Mesures : [x, y, theta] depuis ArUco
- Prédiction : modèle à vitesse constante

Lisse les détections ArUco bruitées et estime les vitesses.

Logs : [KALMAN] RobotX state: x=X, y=Y, theta=T, vx=VX, vy=VY
"""

import numpy as np
from typing import Tuple


class KalmanFilter:
    """
    Filtre de Kalman Étendu pour l'estimation de pose et vitesse du robot 2D.
    
    Vecteur d'état : [x, y, vx, vy, theta, omega]
    """
    
    def __init__(self, dt: float = 1/30.0):
        """
        Initialise le filtre de Kalman.
        
        Args:
            dt: Pas de temps (défaut 30 FPS = 0.033s)
        """
        self.dt = dt
        
        # État : [x, y, vx, vy, theta, omega]
        self.state = np.zeros(6)
        
        # Covariance de l'état
        self.P = np.eye(6) * 1.0
        
        # Bruit de processus
        self.Q = np.diag([0.01, 0.01, 0.1, 0.1, 0.01, 0.1])
        
        # Bruit de mesure
        self.R = np.diag([0.05, 0.05, 0.1])  # [x, y, theta]
        
    def predict(self):
        """
        Étape de prédiction : propage l'état vers l'avant.
        
        Transition d'état :
            x += vx * dt
            y += vy * dt
            vx (constant)
            vy (constant)
            theta += omega * dt
            omega (constant)
        """
        # Matrice de transition d'état
        F = np.array([
            [1, 0, self.dt, 0, 0, 0],
            [0, 1, 0, self.dt, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, self.dt],
            [0, 0, 0, 0, 0, 1]
        ])
        
        # Prédit l'état
        self.state = F @ self.state
        
        # Normalise theta
        self.state[4] = np.arctan2(np.sin(self.state[4]), np.cos(self.state[4]))
        
        # Prédit la covariance
        self.P = F @ self.P @ F.T + self.Q
        
    def update(self, measurement: Tuple[float, float, float]):
        """
        Étape de mise à jour : incorpore la mesure.
        
        Args:
            measurement: (x, y, theta) depuis la détection ArUco
        """
        # Matrice de mesure (observe x, y, theta)
        H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0]
        ])
        
        z = np.array(measurement)
        
        # Innovation
        y = z - H @ self.state
        
        # Normalise l'innovation angulaire
        y[2] = np.arctan2(np.sin(y[2]), np.cos(y[2]))
        
        # Covariance de l'innovation
        S = H @ self.P @ H.T + self.R
        
        # Gain de Kalman
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Met à jour l'état
        self.state = self.state + K @ y
        
        # Normalise theta
        self.state[4] = np.arctan2(np.sin(self.state[4]), np.cos(self.state[4]))
        
        # Met à jour la covariance
        self.P = (np.eye(6) - K @ H) @ self.P
        
    def get_pose(self) -> Tuple[float, float, float]:
        """
        Obtient l'estimation de pose actuelle.
        
        Returns:
            (x, y, theta)
        """
        return (self.state[0], self.state[1], self.state[4])
    
    def get_velocity(self) -> Tuple[float, float, float]:
        """
        Obtient l'estimation de vitesse actuelle.
        
        Returns:
            (vx, vy, omega)
        """
        return (self.state[2], self.state[3], self.state[5])
    
    def get_full_state(self) -> np.ndarray:
        """
        Obtient le vecteur d'état complet.
        
        Returns:
            [x, y, vx, vy, theta, omega]
        """
        return self.state.copy()
    
    def reset(self, initial_pose: Tuple[float, float, float]):
        """
        Réinitialise le filtre avec une nouvelle pose initiale.
        
        Args:
            initial_pose: (x, y, theta)
        """
        self.state = np.array([
            initial_pose[0],  # x
            initial_pose[1],  # y
            0.0,              # vx
            0.0,              # vy
            initial_pose[2],  # theta
            0.0               # omega
        ])
        
        self.P = np.eye(6) * 1.0
