from .base_task import BaseTask
from .navigation import NavigationTask
import numpy as np
import pygame
from core.ia.tactical import TacticalAnalyzer

class FleeTask(NavigationTask):
    """
    Tâche de Fuite : Trouve une zone sûre et s'y rend.
    Basé sur component_test/run_away.py
    Hérite de NavigationTask pour réutiliser le code de déplacement A*.
    """
    def __init__(self):
        super().__init__(target_type='FIXED') # On fixe la cible manuellement
        self.name = "Survival"
        self.tactical = None
        self.safe_zone = None
        
    def execute(self, context):
        world = context.get('world')
        enemy_pose = context.get('human_pose')
        
        if self.tactical is None and world:
            self.tactical = TacticalAnalyzer(world)
            
        # Mise à jour de la zone sûre (périodique)
        # On utilise le même timer que la nav
        if self.replan_timer == 0 or self.safe_zone is None:
            if self.tactical and enemy_pose:
                new_zone = self.tactical.find_safest_zone(enemy_pose)
                if new_zone:
                    if self.safe_zone:
                        # Hysteresis: Only switch if significantly better
                        current_dist = np.hypot(self.safe_zone[0] - enemy_pose[0], self.safe_zone[1] - enemy_pose[1])
                        new_dist = np.hypot(new_zone[0] - enemy_pose[0], new_zone[1] - enemy_pose[1])
                        
                        if new_dist > current_dist + 0.20: # 20cm improvement required
                            self.safe_zone = new_zone
                    else:
                        self.safe_zone = new_zone
                
        # Hack pour utiliser NavigationTask : on surcharge la logique de "Calcul Cible"
        # NavigationTask.execute va appeler son planner vers 'target_pos'
        # On va ruser en copiant le code de NavigationTask mais avec notre cible
        
        # Pour faire propre, on réutilise simplement la logique de NavigationTask
        # mais on injecte notre cible dans un "faux" context via une propriété
        # Ou mieux, on réimplémente juste la partie target selection.
        
        # Approche "Override" :
        # On définit target_pos pour que le replanner de NavigationTask l'utilise
        # Mais NavigationTask recalcule target_pos localement si target_type='ENEMY'
        # Ici target_type='FIXED', donc on doit gérer la mise à jour de self.current_target ? 
        # Non, NavigationTask est un peu rigide. Réimplémentons Execute proprement.
        
        ai_pose = context.get('ai_pose')
        cmd = {'v': 0.0, 'w': 0.0, 'status': 'RUNNING'}
        
        if not ai_pose or not self.safe_zone:
            cmd['status'] = 'SEARCHING_ZONE'
            return cmd

        # Init Planner
        if hasattr(world, 'grid') and self.planner is None:
             from core.ia.planners.a_star import AStarPlanner
             self.planner = AStarPlanner(world.grid, heuristic='euclidean')

        # Replan
        self.replan_timer += 1
        if self.replan_timer > 30 or not self.current_path:
            self.replan_timer = 0
            if self.planner:
                path = self.planner.plan(ai_pose[:2], self.safe_zone)
                if path:
                    self.current_path = path[::2] if len(path) > 4 else path
                    if self.current_path[-1] != self.safe_zone:
                         self.current_path.append(self.safe_zone)
                    self.current_wp_idx = 0
        
        # Suivi (copié de NavigationTask)
        if self.current_path and self.current_wp_idx < len(self.current_path):
            wp = self.current_path[self.current_wp_idx]
            dist_wp = np.hypot(ai_pose[0]-wp[0], ai_pose[1]-wp[1])
            if dist_wp < 0.05: # Aligned with NavigationTask
                self.current_wp_idx += 1
                if self.current_wp_idx >= len(self.current_path):
                    cmd['status'] = 'SAFE'
                    return cmd
            
            future_path = self.current_path[self.current_wp_idx:]
            v, w = self.controller.compute_control(ai_pose, future_path)
            # Fuite : on peut aller un peu plus vite ?
            cmd['v'] = v
            cmd['w'] = w
            
        cmd['debug_info'] = {'path': self.current_path, 'safe_zone': self.safe_zone}
        return cmd

    def draw(self, surface, tm, debug_info):
        super().draw(surface, tm, debug_info)
        sz = debug_info.get('safe_zone')
        if sz:
            px = tm.world_to_projector(sz[0], sz[1])
            pygame.draw.circle(surface, (0, 255, 0), px, 40, 2) # Grand cercle vert

if __name__ == '__main__':
    FleeTask.run_standalone()
