"""Classe abstraite de contrôleur de base."""

from abc import ABC, abstractmethod
from typing import Tuple, Dict, Any
import logging

logger = logging.getLogger(__name__)


class BaseController(ABC):
    """
    Classe de base abstraite pour tous les contrôleurs de robot.
    
    Les contrôleurs traduisent les entrées (clavier, joystick, etc.) 
    en commandes de vitesse pour le robot (linéaire, angulaire).
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialise le contrôleur avec la configuration.
        
        Args:
            config: Dictionnaire de configuration spécifique au contrôleur
        """
        self.config = config
        self.linear_cmd = 0.0
        self.angular_cmd = 0.0
        self._enabled = True
        
        logger.info(f"{self.__class__.__name__} initialisé")
    
    @abstractmethod
    def update(self, events: list) -> None:
        """
        Met à jour l'état du contrôleur basé sur les événements d'entrée.
        
        Args:
            events: Liste des événements pygame à traiter
        """
        pass
    
    @abstractmethod
    def get_command(self) -> Tuple[float, float]:
        """
        Obtient la commande de vitesse actuelle.
        
        Returns:
            Tuple de (vitesse_lineaire, vitesse_angulaire)
        """
        pass
    
    @abstractmethod
    def emergency_stop(self) -> None:
        """Exécute l'arrêt d'urgence - met immédiatement toutes les vitesses à zéro."""
        pass
    
    def enable(self) -> None:
        """Active le contrôleur."""
        self._enabled = True
        logger.info(f"{self.__class__.__name__} activé")
    
    def disable(self) -> None:
        """Désactive le contrôleur et arrête le robot."""
        self._enabled = False
        self.emergency_stop()
        logger.info(f"{self.__class__.__name__} désactivé")
    
    def is_enabled(self) -> bool:
        """Vérifie si le contrôleur est activé."""
        return self._enabled
    
    def reset(self) -> None:
        """Réinitialise le contrôleur à l'état initial."""
        self.linear_cmd = 0.0
        self.angular_cmd = 0.0
        logger.debug(f"{self.__class__.__name__} réinitialisé")
