#!/usr/bin/env python3
"""
Module Visual Robot
Gère la représentation visuelle du robot dans l'interface
"""

import pygame

import math


class VisualRobot:
    """Robot visuel pour la simulation dans l'interface"""
    
    def __init__(self, x, y, angle=0, config=None):
        self.x = x
        self.y = y
        self.angle = angle  # en degrés
        
        # Config is already the 'visual' section, use directly
        visual_cfg = config if config else {}
        
        self.max_trail_length = visual_cfg.get('trail_length', 80)
        self.trail = []  # Traînée
        
        # Position de repos (centre)
        self.rest_x = x
        self.rest_y = y
        self.rest_angle = angle
        
        # Pour le retour au centre
        self.return_speed = visual_cfg.get('return_speed', 0.15)
        
        # Échelles
        self.speed_scale = visual_cfg.get('speed_scale', 1000)
        self.angular_scale = visual_cfg.get('angular_scale', 5)
        self.max_distance = visual_cfg.get('max_distance', 220)
    
    def update(self, linear_x, angular_z, dt=0.016):
        """Met à jour la position du robot selon les commandes"""
        # Si aucune commande, retour progressif au centre
        if abs(linear_x) < 0.01 and abs(angular_z) < 0.01:
            # Retour en position
            dx = self.rest_x - self.x
            dy = self.rest_y - self.y
            da = self.rest_angle - self.angle
            
            # Normaliser l'angle
            while da > 180:
                da -= 360
            while da < -180:
                da += 360
            
            self.x += dx * self.return_speed
            self.y += dy * self.return_speed
            self.angle += da * self.return_speed
            
            # Snap au centre si très proche
            if abs(dx) < 1 and abs(dy) < 1 and abs(da) < 1:
                self.x = self.rest_x
                self.y = self.rest_y
                self.angle = self.rest_angle
                self.trail.clear()
        else:

            # Simulation du mouvement
            # Rotation
            angular_deg = math.degrees(angular_z)
            self.angle += angular_deg * self.angular_scale * dt
            
            # Translation
            angle_rad = math.radians(self.angle)
            dx = math.sin(angle_rad) * linear_x * self.speed_scale * dt
            dy = -math.cos(angle_rad) * linear_x * self.speed_scale * dt
            
            self.x += dx
            self.y += dy
            
            # Limiter la zone de déplacement (pas trop loin du centre)
            dist = math.sqrt((self.x - self.rest_x)**2 + (self.y - self.rest_y)**2)
            if dist > self.max_distance:
                ratio = self.max_distance / dist
                self.x = self.rest_x + (self.x - self.rest_x) * ratio
                self.y = self.rest_y + (self.y - self.rest_y) * ratio
            
            # Ajouter à la traînée
            if len(self.trail) == 0 or \
               (abs(self.trail[-1][0] - self.x) > 3 or abs(self.trail[-1][1] - self.y) > 3):
                self.trail.append((int(self.x), int(self.y)))
                
                if len(self.trail) > self.max_trail_length:
                    self.trail.pop(0)
    
    def draw(self, surface, scale=1.0):
        """Dessine le robot avec mise à l'échelle"""
        # Traînée avec gradient
        if len(self.trail) > 1:
            for i in range(len(self.trail) - 1):
                alpha = i / len(self.trail)
                color = (
                    int(0 * alpha),
                    int(150 * alpha),
                    int(150 * alpha)
                )
                radius = int((2 + alpha * 2) * scale)
                pygame.draw.circle(surface, color, self.trail[i], max(1, radius))
        
        # Tailles avec échelle
        rayon_ext = int(32 * scale)
        rayon_int = int(28 * scale)
        rayon_roue = int(8 * scale)
        rayon_roue_int = int(6 * scale)
        longueur_fleche = int(40 * scale)
        
        # Corps du robot
        pos = (int(self.x), int(self.y))
        pygame.draw.circle(surface, (40, 40, 50), pos, rayon_ext)
        pygame.draw.circle(surface, (0, 200, 255), pos, rayon_int, max(1, int(3 * scale)))
        
        # Roues
        angle_rad = math.radians(self.angle)
        for offset in [-15 * scale, 15 * scale]:
            angle_roue = angle_rad + math.radians(90)
            rx = self.x + math.cos(angle_roue) * offset
            ry = self.y + math.sin(angle_roue) * offset
            pygame.draw.circle(surface, (100, 100, 100), (int(rx), int(ry)), rayon_roue)
            pygame.draw.circle(surface, (60, 60, 60), (int(rx), int(ry)), rayon_roue_int)
        
        # Flèche directionnelle
        x1 = self.x + math.sin(angle_rad) * longueur_fleche
        y1 = self.y - math.cos(angle_rad) * longueur_fleche
        
        # Corps de la flèche
        pygame.draw.line(surface, (255, 100, 100), pos, (x1, y1), max(1, int(4 * scale)))
        
        # Pointe
        for angle_offset in [-25, 25]:
            angle_pointe = angle_rad + math.radians(180 + angle_offset)
            x2 = x1 + math.sin(angle_pointe) * 15 * scale
            y2 = y1 - math.cos(angle_pointe) * 15 * scale
            pygame.draw.line(surface, (255, 100, 100), (x1, y1), (x2, y2), max(1, int(4 * scale)))
        
        # Point central
        pygame.draw.circle(surface, (255, 255, 0), pos, max(1, int(3 * scale)))
    
    def reset_trail(self):
        """Efface la traînée"""
        self.trail.clear()
    
    def reset_position(self):
        """Remet le robot au centre instantanément"""
        self.x = self.rest_x
        self.y = self.rest_y
        self.angle = self.rest_angle
        self.trail.clear()