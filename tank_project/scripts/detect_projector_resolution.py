#!/usr/bin/env python3
"""
Détecter la résolution du projecteur

Ce script aide à détecter la résolution réelle du projecteur.
Il affiche les moniteurs disponibles et leurs résolutions.

Usage :
    python3 scripts/detect_projector_resolution.py
"""

import pygame
import sys

def main():
    print("=" * 60)
    print("  DÉTECTEUR DE RÉSOLUTION DU PROJECTEUR")
    print("=" * 60)
    print()
    
    pygame.init()
    
    # Obtenir toutes les tailles d'affichage disponibles
    # Note: Sur certains systèmes Linux multi-écrans, get_desktop_sizes
    # peut retourner la taille combinée ou des tailles individuelles selon le driver.
    try:
        displays = pygame.display.get_desktop_sizes()
    except AttributeError:
        # Fallback pour vieilles versions de pygame
        info = pygame.display.Info()
        displays = [(info.current_w, info.current_h)]
        print("Note: Pygame ancien détecté, affichage de l'écran principal uniquement.")
    
    print(f"Détecté {len(displays)} affichage(s) signalés par l'OS :\n")
    
    for i, (width, height) in enumerate(displays):
        display_name = "Moniteur Principal" if i == 0 else f"Affichage Secondaire {i}"
        print(f"  Affichage {i} : {width} x {height} px ({display_name})")
    
    print()
    print("=" * 60)
    print()
    
    # Test : Créer une fenêtre et montrer sa taille réelle
    print("Création d'une fenêtre de test...")
    print("1. La fenêtre s'ouvrira sur votre affichage principal")
    print("2. Faites-la glisser vers le projecteur")
    print("3. Appuyez sur F11 pour passer en plein écran")
    print("4. Vérifiez si les infos affichées correspondent à la réalité")
    print()
    
    # Créer fenêtre redimensionnable
    screen = pygame.display.set_mode((800, 600), pygame.RESIZABLE)
    pygame.display.set_caption("Test Résolution Projecteur - Glissez vers projecteur, F11")
    
    font_large = pygame.font.SysFont("Arial", 48)
    font_small = pygame.font.SysFont("Arial", 32)
    
    running = True
    is_fullscreen = False
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_F11:
                    # Basculer plein écran
                    if is_fullscreen:
                        screen = pygame.display.set_mode((800, 600), pygame.RESIZABLE)
                        is_fullscreen = False
                    else:
                        screen = pygame.display.set_mode((0, 0), pygame.NOFRAME | pygame.FULLSCREEN)
                        is_fullscreen = True
                elif event.key == pygame.K_ESCAPE:
                    if is_fullscreen:
                        screen = pygame.display.set_mode((800, 600), pygame.RESIZABLE)
                        is_fullscreen = False
                    else:
                        running = False
                elif event.key == pygame.K_q:
                    running = False
        
        # Effacer écran
        screen.fill((30, 30, 50))
        
        # Obtenir taille fenêtre actuelle
        width, height = screen.get_size()
        
        # Afficher infos
        mode_text = "MODE PLEIN ÉCRAN" if is_fullscreen else "MODE FENÊTRÉ"
        mode_surface = font_large.render(mode_text, True, (255, 255, 100))
        mode_rect = mode_surface.get_rect(center=(width // 2, height // 3))
        screen.blit(mode_surface, mode_rect)
        
        # Afficher résolution
        res_text = f"Résolution Actuelle : {width} x {height} px"
        res_surface = font_large.render(res_text, True, (255, 255, 255))
        res_rect = res_surface.get_rect(center=(width // 2, height // 2))
        screen.blit(res_surface, res_rect)
        
        instructions = [
            "F11 - Basculer Plein Écran",
            "ESC - Quitter Plein Écran / Quitter",
            "Q - Quitter"
        ]
        
        y_offset = height // 2 + 80
        for instruction in instructions:
            inst_surface = font_small.render(instruction, True, (200, 200, 200))
            inst_rect = inst_surface.get_rect(center=(width // 2, y_offset))
            screen.blit(inst_surface, inst_rect)
            y_offset += 40
        
        # Note en bas
        if is_fullscreen:
            note = "NOTEZ CETTE RÉSOLUTION POUR VOTRE CONFIG !"
            note_color = (100, 255, 100)
        else:
            note = "Glissez la fenêtre vers le projecteur, puis F11"
            note_color = (255, 200, 100)
        
        note_surface = font_small.render(note, True, note_color)
        note_rect = note_surface.get_rect(center=(width // 2, height - 50))
        screen.blit(note_surface, note_rect)
        
        pygame.display.flip()
    
    pygame.quit()
    print("Fin du test.")

if __name__ == "__main__":
    main()