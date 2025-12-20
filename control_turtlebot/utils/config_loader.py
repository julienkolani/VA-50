"""Utilitaire de chargement de configuration pour les fichiers YAML."""

import yaml
from pathlib import Path
from typing import Dict, Any
import logging

logger = logging.getLogger(__name__)


class ConfigLoader:
    """Charge et gère les fichiers de configuration YAML."""
    
    def __init__(self, config_dir: Path = None):
        """
        Initialise le chargeur de configuration.
        
        Args:
            config_dir: Répertoire contenant les fichiers config. Si None, utilise ./config
        """
        if config_dir is None:
            # Obtient le répertoire config relatif à l'emplacement de ce fichier
            self.config_dir = Path(__file__).parent.parent / 'config'
        else:
            self.config_dir = Path(config_dir)
        
        if not self.config_dir.exists():
            raise FileNotFoundError(f"Répertoire config introuvable : {self.config_dir}")
        
        logger.info(f"Répertoire config : {self.config_dir}")
    
    def load(self, config_name: str) -> Dict[str, Any]:
        """
        Charge un fichier de configuration YAML.
        
        Args:
            config_name: Nom du fichier config (sans extension .yaml)
            
        Returns:
            Dictionnaire contenant la configuration
            
        Raises:
            FileNotFoundError: Si le fichier config n'existe pas
            yaml.YAMLError: Si le fichier config est malformé
        """
        config_path = self.config_dir / f"{config_name}.yaml"
        
        if not config_path.exists():
            raise FileNotFoundError(f"Fichier config introuvable : {config_path}")
        
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            logger.info(f"Config chargée : {config_name}")
            return config
        except yaml.YAMLError as e:
            logger.error(f"Erreur parsing {config_name}.yaml : {e}")
            raise
    
    def load_all(self) -> Dict[str, Dict[str, Any]]:
        """
        Charge tous les fichiers de configuration YAML du répertoire config.
        
        Returns:
            Dictionnaire mappant les noms de config à leur contenu
        """
        all_configs = {}
        
        for config_file in self.config_dir.glob("*.yaml"):
            config_name = config_file.stem
            try:
                all_configs[config_name] = self.load(config_name)
            except Exception as e:
                logger.warning(f"Échec du chargement de {config_name} : {e}")
        
        return all_configs
    
    def get_value(self, config_name: str, key_path: str, default: Any = None) -> Any:
        """
        Obtient une valeur spécifique d'une config en utilisant la notation par points.
        
        Args:
            config_name: Nom du fichier config
            key_path: Chemin vers la valeur (ex: 'websocket.uri')
            default: Valeur par défaut si la clé n'est pas trouvée
            
        Returns:
            Valeur à key_path ou default
            
        Example:
            loader.get_value('network', 'websocket.uri')  # Retourne 'ws://localhost:8765'
        """
        config = self.load(config_name)
        
        keys = key_path.split('.')
        value = config
        
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default
        
        return value
