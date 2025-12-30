#!/usr/bin/env python3
"""
Tank Arena - Main Entry Point
Lance le GameManager qui orchestre:
- Perception (Realsense/ArUco)
- IA (Behavior Tree)
- Rendu (Pygame)
- Contrôle (ROS Bridge)
"""

import sys
import argparse
from pathlib import Path

# Add project root to path
ROOT_DIR = Path(__file__).parent.resolve()
sys.path.insert(0, str(ROOT_DIR))

from core.game.game_manager import GameManager

def main():
    parser = argparse.ArgumentParser(description="Tank Arena - Unified Game Launcher")
    parser.add_argument('--mock', action='store_true', help="Force Mock Mode (Simulation sans hardware)")
    args = parser.parse_args()

    print("==========================================")
    print("   TANK ARENA - AUTO MODE INITIALIZED     ")
    print("==========================================")
    print(f"Mode: {'MOCK/SIMULATION' if args.mock else 'REAL HARDWARE'}")
    
    try:
        game = GameManager(mock=args.mock)
        game.run()
    except Exception as e:
        print(f"[MAIN] Critical Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
