from .base_task import BaseTask
import numpy as np
import pygame
import time

class AttackTask(BaseTask):
    """
    Tâche d'Attaque (Sniper) : Tourne sur place et tire si aligné.
    Basé sur component_test/point_and_tir.py
    """
    def __init__(self):
        super().__init__("Combat")
        self.align_precision = 3.0 # Degrés
        self.rotation_gain = 1.5
        self.last_shot_time = 0
        self.shoot_cooldown = 1.5
        
    def execute(self, context):
        ai_pose = context.get('ai_pose')
        enemy_pose = context.get('human_pose')
        
        cmd = {'v': 0.0, 'w': 0.0, 'fire_request': False, 'status': 'AIMING'}
        
        if not ai_pose or not enemy_pose:
            cmd['status'] = 'NO_TARGET'
            return cmd
            
        # Calcul angle
        dx = enemy_pose[0] - ai_pose[0]
        dy = enemy_pose[1] - ai_pose[1]
        desired_theta = np.arctan2(dy, dx)
        
        error_theta = desired_theta - ai_pose[2]
        # Normalize
        error_theta = (error_theta + np.pi) % (2 * np.pi) - np.pi
        error_deg = abs(np.degrees(error_theta))
        
        # Commande P (Signe inverse comme dans point_and_tir.py)
        w = np.clip(-self.rotation_gain * error_theta, -1.0, 1.0)
        
        # Deadband & Tir
        if error_deg < self.align_precision:
            w = 0.0 # Stop pour tirer
            cmd['status'] = 'LOCKED'
            
            # Gestion Tir
            current_time = time.time()
            if (current_time - self.last_shot_time) > self.shoot_cooldown:
                cmd['fire_request'] = True
                self.last_shot_time = current_time
                print("[TASK] FEU !")
        
        cmd['w'] = w
        # Pass poses for drawing
        cmd['debug_info'] = {
            'locked': (error_deg < self.align_precision), 
            'error': error_deg,
            'ai_pose': ai_pose,
            'enemy_pose': enemy_pose
        }
        return cmd

    def draw(self, surface, tm, debug_info):
        locked = debug_info.get('locked', False)
        ai_pose = debug_info.get('ai_pose')
        enemy_pose = debug_info.get('enemy_pose')
        error_deg = debug_info.get('error', 0.0)
        
        color = (0, 255, 0) if locked else (255, 0, 0)
        
        # Dessin Laser (Ligne de visée)
        if ai_pose and enemy_pose:
            p1 = tm.world_to_projector(ai_pose[0], ai_pose[1])
            p2 = tm.world_to_projector(enemy_pose[0], enemy_pose[1])
            pygame.draw.line(surface, color, p1, p2, 2 if locked else 1)
            
            # Petit réticule sur l'ennemi
            pygame.draw.circle(surface, color, p2, 5, 1)

        # Indicateur HUD
        font = pygame.font.SysFont("Impact", 20)
        
        lines = [
            f"STATUS: {'LOCKED' if locked else 'AIMING'}",
            f"ERROR:  {error_deg:.1f} deg"
        ]
        
        y = 40
        for line in lines:
            txt = font.render(line, True, color)
            surface.blit(txt, (10, y))
            y += 25

if __name__ == '__main__':
    AttackTask.run_standalone()
