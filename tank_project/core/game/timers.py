"""
Chronomètres - Gestion du Temps de Recharge & Match

Gère toutes les mécaniques de jeu basées sur le temps :
- Chronomètre de match (total écoulé, restant)
- Temps de recharge des tirs par robot
- Buffs/debuffs temporaires (futur)

Tous les temps sont en secondes (float).
Utilise l'heure système (time.time()) pour le chronométrage absolu.
"""

import time


class Timer:
    """Compte à rebours simple."""
    
    def __init__(self, duration_seconds):
        self.duration = duration_seconds
        self.start_time = None
        
    def start(self):
        """Démarre le chronomètre."""
        self.start_time = time.time()
        
    def elapsed(self):
        """Obtient le temps écoulé en secondes."""
        if self.start_time is None:
            return 0.0
        return time.time() - self.start_time
    
    def remaining(self):
        """Obtient le temps restant en secondes."""
        return max(0.0, self.duration - self.elapsed())
    
    def is_expired(self):
        """Vérifie si le chronomètre est expiré."""
        return self.elapsed() >= self.duration
    
    def reset(self):
        """Réinitialise le chronomètre au début."""
        self.start_time = None


class CooldownManager:
    """
    Gère les temps de recharge des tirs pour les deux robots.
    
    Suit quand chaque robot a tiré pour la dernière fois et quand il peut tirer à nouveau.
    """
    
    def __init__(self, ai_cooldown_sec, human_cooldown_sec):
        """
        Initialise le gestionnaire de temps de recharge.
        
        Args:
            ai_cooldown_sec: Durée de recharge robot IA
            human_cooldown_sec: Durée de recharge robot Humain
        """
        self.ai_cooldown = ai_cooldown_sec
        self.human_cooldown = human_cooldown_sec
        
        self.last_shot_ai = 0.0
        self.last_shot_human = 0.0
        
    def can_shoot_ai(self):
        """Vérifie si l'IA peut tirer maintenant."""
        return time.time() >= self.last_shot_ai + self.ai_cooldown
    
    def can_shoot_human(self):
        """Vérifie si l'humain peut tirer maintenant."""
        return time.time() >= self.last_shot_human + self.human_cooldown
    
    def register_shot_ai(self):
        """
        Enregistre que l'IA a tiré.
        
        Logs :
            [TIMER] Recharge IA commencée (X s restantes)
        """
        self.last_shot_ai = time.time()
        
    def register_shot_human(self):
        """
        Enregistre que l'humain a tiré.
        
        Logs :
            [TIMER] Recharge Humain commencée (X s restantes)
        """
        self.last_shot_human = time.time()
    
    def time_until_next_shot_ai(self):
        """Obtient les secondes jusqu'au prochain tir IA."""
        return max(0.0, self.last_shot_ai + self.ai_cooldown - time.time())
    
    def time_until_next_shot_human(self):
        """Obtient les secondes jusqu'au prochain tir Humain."""
        return max(0.0, self.last_shot_human + self.human_cooldown - time.time())
