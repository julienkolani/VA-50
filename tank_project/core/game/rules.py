"""
Règles du Jeu - Configuration & Constantes

Définit tous les paramètres et règles du jeu :
- Temps de recharge des tirs (IA vs Humain)
- Durée du match
- Conditions de victoire
- Règles de score

Toutes les valeurs sont chargées depuis config/game.yaml.
Ce module fournit la validation et les valeurs par défaut.
"""

from dataclasses import dataclass


@dataclass
class GameRules:
    """
    Règles et paramètres du jeu.
    
    Chargé depuis config/game.yaml mais fournit des valeurs par défaut raisonnables.
    """
    
    # Chronométrage du match
    match_duration_seconds: float = 180.0  # Défaut 3 minutes
    
    # Temps de recharge des tirs
    human_shot_cooldown: float = 5.0  # L'humain peut tirer toutes les 5 secondes
    ai_shot_cooldown: float = 3.0     # L'IA peut tirer toutes les 3 secondes
    
    # Conditions de victoire
    max_hits_to_win: int = 10         # Le premier à 10 touches gagne
    sudden_death: bool = False        # Continuer après l'expiration du temps ?
    
    # Mécanique de tir
    shot_range_meters: float = 5.0    # Portée effective maximale
    shot_speed_mps: float = 10.0      # Vitesse du tir (pour l'animation)
    
    @classmethod
    def from_config(cls, config_dict):
        """
        Crée les règles à partir du dictionnaire de configuration.
        
        Args:
            config_dict: YAML analysé depuis config/game.yaml['match']
            
        Returns:
            Instance GameRules avec valeurs validées
        """
        # Map config keys to class attributes
        mapped = {}
        
        if 'duration_seconds' in config_dict:
            mapped['match_duration_seconds'] = config_dict['duration_seconds']
        if 'human_shot_seconds' in config_dict:
            mapped['human_shot_cooldown'] = config_dict['human_shot_seconds']
        if 'ai_shot_seconds' in config_dict:
            mapped['ai_shot_cooldown'] = config_dict['ai_shot_seconds']
        if 'max_hits' in config_dict:
            mapped['max_hits_to_win'] = config_dict['max_hits']
        if 'sudden_death' in config_dict:
            mapped['sudden_death'] = config_dict['sudden_death']
        if 'range_m' in config_dict:
            mapped['shot_range_meters'] = config_dict['range_m']
        if 'speed_mps' in config_dict:
            mapped['shot_speed_mps'] = config_dict['speed_mps']
        
        return cls(**mapped)
    
    def validate(self):
        """
        Valide la cohérence des règles.
        
        Raises:
            ValueError: Si les règles sont incohérentes ou invalides
        """
        if self.match_duration_seconds <= 0:
            raise ValueError("Match duration must be positive")
        if self.human_shot_cooldown <= 0 or self.ai_shot_cooldown <= 0:
            raise ValueError("Cooldowns must be positive")
        if self.max_hits_to_win < 1:
            raise ValueError("Max hits must be >= 1")
