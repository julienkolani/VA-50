#!/usr/bin/env python3
"""
TurtleBot Controller - Point d'Entrée Principal

Interface professionnelle basée sur Pygame pour le contrôle manuel du robot via WebSocket.
"""

import sys
import logging
from pathlib import Path

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] %(name)s: %(message)s'
)

logger = logging.getLogger(__name__)


def main():
    """Point d'entrée principal."""
    try:
        # Add current directory to path for imports
        sys.path.insert(0, str(Path(__file__).parent))
        
        # Import after path setup
        from utils.config_loader import ConfigLoader
        
        logger.info("=" * 60)
        logger.info(" TurtleBot Controller")
        logger.info("=" * 60)
        
        # Charge toutes les configurations
        logger.info("Chargement de la configuration...")
        config_loader = ConfigLoader()
        
        try:
            all_config = config_loader.load_all()
            logger.info(f"Chargé {len(all_config)} fichiers de configuration")
        except Exception as e:
            logger.error(f"Échec du chargement de la configuration : {e}")
            logger.error("Veuillez vérifier que le répertoire config/ contient des fichiers YAML valides")
            return 1
        
        # Import and launch UI
        from integrated_ui import IntegratedUI
        
        logger.info("Configuration chargée avec succès !")
        logger.info("")
        logger.info("Résumé de la configuration :")
        for name, cfg in all_config.items():
            logger.info("  - %s.yaml: %d clés de premier niveau", name, len(cfg))
        
        logger.info("")
        logger.info("Lancement de l'interface intégrée...")
        
        # Obtient la configuration UI
        ui_cfg = all_config.get('ui', {})
        network_cfg = all_config.get('network', {})
        
        # Construit l'URI WebSocket depuis la config (network.yaml a 'uri' directement)
        ws_cfg = network_cfg.get('websocket', {})
        ws_uri = ws_cfg.get('uri', 'ws://localhost:8765')
        
        # Crée et lance l'interface
        ui = IntegratedUI(
            width=ui_cfg.get('default_width', 1400),
            height=ui_cfg.get('default_height', 900),
            ws_uri=ws_uri
        )
        ui.run()
        
        logger.info("Interface fermée normalement")
        return 0
        
    except KeyboardInterrupt:
        logger.info("Interrompu par l'utilisateur (Ctrl+C)")
        return 0
        
    except Exception as e:
        logger.error(f"Erreur fatale : {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())