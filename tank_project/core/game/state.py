"""
État du Jeu - Statut Complet du Jeu

Maintient l'état complet du match en cours :
- Scores des robots (touches données/reçues)
- Chronomètre du match (écoulé, restant)
- Chronomètres de recharge (prochain tir autorisé pour chaque robot)
- Statut du jeu (calibration, jeu, pause, fini)
- Information sur le vainqueur

Ceci est une structure de données pure sans logique.
Toutes les modifications d'état sont faites par game_engine.py.
"""

from dataclasses import dataclass
from enum import Enum

class GameStatus(Enum):
    """États du cycle de vie du jeu"""
    CALIBRATION = "calibration"
    READY = "ready"
    PLAYING = "playing"
    PAUSED = "paused"
    FINISHED = "finished"


@dataclass
class RobotScore:
    """Information de score par robot"""
    robot_id: int
    hits_inflicted: int = 0  # Touches marquées par ce robot sur l'ennemi
    hits_received: int = 0   # Touches reçues par ce robot
    shots_fired: int = 0     # Total des tirs tentés
    

@dataclass
class GameState:
    """
    Instantané complet de l'état du jeu.
    
    Ceci est la source unique de vérité pour le statut du jeu.
    Immuable entre les ticks - game_engine crée un nouvel état à chaque tick.
    """
    status: GameStatus
    
    # Suivi du temps
    match_start_time: float  # Timestamp Unix
    current_time: float      # Timestamp Unix
    match_duration: float    # Durée totale match en secondes
    
    # Scores
    robot_4_score: RobotScore  # Robot IA
    robot_5_score: RobotScore  # Robot Humain
    
    # Temps de recharge (Timestamps Unix)
    next_shot_robot_4: float
    next_shot_robot_5: float
    
    # Info vainqueur (None si en cours)
    winner: str = None  # "AI", "HUMAN", ou "DRAW"
    
    @property
    def time_remaining(self):
        """Calcule le temps restant en secondes."""
        elapsed = self.current_time - self.match_start_time
        return max(0, self.match_duration - elapsed)
    
    @property
    def can_shoot_ai(self):
        """Vérifie si le cooldown IA permet de tirer."""
        return self.current_time >= self.next_shot_robot_4
    
    @property
    def can_shoot_human(self):
        """Vérifie si le cooldown humain permet de tirer."""
        return self.current_time >= self.next_shot_robot_5
