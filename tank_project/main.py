#!/usr/bin/env python3
"""
"""
Tank Arena - Point d'Entrée Principal

Point d'entrée unique pour le projet tank arena.

Usage:
    python3 main.py [mode]
    
    Modes:
    - game        : Lancer le jeu (défaut)
    - calibration : Lancer l'assistant de calibration
    - export      : Exporter données debug
    
Exemples:
    python3 main.py
    python3 main.py game
    python3 main.py calibration
    python3 main.py export
    
Ou en tant que module:
    python3 -m tank_project.main
    cd /home/julien/ros2_ws/src && python3 -m tank_project.main
"""

import sys
import argparse
from pathlib import Path

# Ajouter le répertoire courant au path
PROJECT_ROOT = Path(__file__).parent
sys.path.insert(0, str(PROJECT_ROOT))


def main():
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(
        description='Tank Arena - Système de combat de chars robotiques',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples:
  %(prog)s                  Lance le jeu
  %(prog)s game             Lance le jeu
  %(prog)s calibration      Lance la calibration
  %(prog)s export           Exporte les données debug
        """
    )
    
    parser.add_argument(
        'mode',
        nargs='?',
        default='game',
        choices=['game', 'calibration', 'export'],
        help='Mode de lancement (défaut: game)'
    )
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("      TANK ARENA - Système Combat Chars Robotiques")
    print("=" * 60)
    print()
    
    # Lancer le mode approprié
    if args.mode == 'calibration':
        print("[MAIN] Mode: CALIBRATION")
        print("-" * 60)
        from perception.calibration.standalone_wizard import main as run_calibration
        run_calibration()
        
    elif args.mode == 'export':
        print("[MAIN] Mode: EXPORT DEBUG DATA")
        print("-" * 60)
        from scripts.export_debug_data import main as run_export
        run_export()
        
    else:  # game
        print("[MAIN] Mode: GAME")
        print("-" * 60)
        from scripts.run_game import main as run_game
        run_game()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n[MAIN] Interruption utilisateur (Ctrl+C)")
        sys.exit(0)
    except Exception as e:
        print("\n[MAIN] ERREUR: {}".format(e))
        import traceback
        traceback.print_exc()
        sys.exit(1)
