"""
Game Rules - Configuration & Constants

Defines all game parameters and rules:
- Shot cooldowns (AI vs Human)
- Match duration
- Win conditions
- Scoring rules

All values are loaded from config/game.yaml.
This module provides validation and defaults.
"""

from dataclasses import dataclass


@dataclass
class GameRules:
    """
    Game rules and parameters.
    
    Loaded from config/game.yaml but provides sensible defaults.
    """
    
    # Match timing
    match_duration_seconds: float = 180.0  # 3 minutes default
    
    # Shot cooldowns
    human_shot_cooldown: float = 5.0  # Human can shoot every 5 seconds
    ai_shot_cooldown: float = 3.0     # AI can shoot every 3 seconds
    
    # Win conditions
    max_hits_to_win: int = 10         # First to 10 hits wins
    sudden_death: bool = False        # Continue after time expires?
    
    # Shot mechanics
    shot_range_meters: float = 5.0    # Maximum effective range
    shot_speed_mps: float = 10.0      # Shot travel speed (for animation)
    
    @classmethod
    def from_config(cls, config_dict):
        """
        Create rules from config dictionary.
        
        Args:
            config_dict: Parsed YAML from config/game.yaml['match']
            
        Returns:
            GameRules instance with validated values
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
        Validate rule consistency.
        
        Raises:
            ValueError: If rules are inconsistent or invalid
        """
        if self.match_duration_seconds <= 0:
            raise ValueError("Match duration must be positive")
        if self.human_shot_cooldown <= 0 or self.ai_shot_cooldown <= 0:
            raise ValueError("Cooldowns must be positive")
        if self.max_hits_to_win < 1:
            raise ValueError("Max hits must be >= 1")
