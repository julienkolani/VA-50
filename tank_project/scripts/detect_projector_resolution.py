#!/usr/bin/env python3
"""
Détecter et Configurer la Résolution du Projecteur

Ce script :
1. Liste les écrans connectés.
2. Vous permet d'identifier le projecteur.
3. Met à jour automatiquement config/projector.yaml.

Usage :
    python3 scripts/detect_projector_resolution.py
"""

import pygame
import sys
import yaml
from pathlib import Path

def load_projector_config():
    config_path = Path(__file__).parent.parent / 'config' / 'projector.yaml'
    if config_path.exists():
        with open(config_path) as f:
            return yaml.safe_load(f), config_path
    return None, config_path

def save_projector_config(config, path):
    with open(path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)
    print(f"✅ Configuration sauvegardée dans {path}")

def main():
    print("=" * 60)
    print("  CONFIGURATION AUTOMATIQUE DU PROJECTEUR")
    print("=" * 60)
    
    pygame.init()
    
    # 1. Détection des écrans
    try:
        displays = pygame.display.get_desktop_sizes()
    except AttributeError:
        # Fallback Pygame ancien
        info = pygame.display.Info()
        displays = [(info.current_w, info.current_h)]
        print("Note: Pygame ancien, détection limitée.")

    print(f"\n🖥️  Écrans détectés : {len(displays)}\n")
    
    for i, (w, h) in enumerate(displays):
        print(f"  [{i}] {w} x {h} px  {'<-- Probablement le PC' if i==0 else '<-- Probablement le Projecteur'}")

    print("\nQuelle est l'ID de votre projecteur ?")
    
    try:
        choice = input(f"Entrez le numéro (0-{len(displays)-1}) ou 'q' pour quitter : ")
        if choice.lower() == 'q':
            return
        
        idx = int(choice)
        if idx < 0 or idx >= len(displays):
            print("❌ ID invalide.")
            return
            
        target_w, target_h = displays[idx]
        print(f"\n👌 Vous avez choisi : {target_w} x {target_h} px")
        
        # 2. Mise à jour de la config
        config, path = load_projector_config()
        if config is None:
            print("❌ Erreur : config/projector.yaml introuvable.")
            return
            
        print(f"\nAncienne configuration : {config['projector']['width']} x {config['projector']['height']}")
        
        # Mise à jour resolution
        config['projector']['width'] = target_w
        config['projector']['height'] = target_h
        
        # Mise à jour offset (si 2 écrans et projecteur est le 2eme)
        # On suppose que l'offset X est la largeur du premier écran si on choisit le 2eme
        if len(displays) > 1 and idx == 1:
            offset_x = displays[0][0]
            print(f"Mise à jour de l'offset X à {offset_x} (largeur écran principal)")
            config['display']['monitor_offset_x'] = offset_x
        
        # Confirmation
        confirm = input("\nSauvegarder cette configuration ? (o/n) : ")
        if confirm.lower() == 'o':
            save_projector_config(config, path)
            print("\n✨ SUCCÈS ! La résolution est corrigée.")
            print("Relancez maintenant 'python3 scripts/run_calibration.py'")
        else:
            print("Annulé.")

    except ValueError:
        print("Entrée invalide.")
    except Exception as e:
        print(f"Erreur : {e}")
    finally:
        pygame.quit()

if __name__ == "__main__":
    main()