from .base_task import BaseTask
import numpy as np
import pygame
from core.ia.planners.a_star import AStarPlanner
from core.control.trajectory_follower import TrajectoryFollower

class NavigationTask(BaseTask):
    """
    Tâche de Navigation : Aller vers une cible (Ennemi ou Point fixe).
    Basé sur component_test/go_to_point.py
    """
    def __init__(self, target_type='ENEMY', safety_dist=0.40):
        super().__init__("Navigation")
        self.target_type = target_type # 'ENEMY' ou 'FIXED'
        self.safety_dist = safety_dist
        self.planner = None
        self.controller = TrajectoryFollower({
            'lookahead_distance_m': 0.15,
            'k_velocity': 0.15,
            'waypoint_threshold_m': 0.05,
            'max_linear_mps': 0.20,
            'max_angular_radps': 1.0
        })
        self.current_path = []
        self.current_wp_idx = 0
        self.replan_timer = 0
        
    def execute(self, context):
        ai_pose = context.get('ai_pose')
        enemy_pose = context.get('human_pose')
        world = context.get('world')
        
        cmd = {'v': 0.0, 'w': 0.0, 'status': 'RUNNING'}
        
        if not ai_pose or not enemy_pose:
            cmd['status'] = 'WAITING_POSE'
            return cmd

        # Init Planner (Lazy)
        if hasattr(world, 'grid') and self.planner is None:
             self.planner = AStarPlanner(world.grid, heuristic='euclidean')

        # 1. Calcul Cible
        target_pos = None
        if self.target_type == 'ENEMY':
            dx, dy = ai_pose[0] - enemy_pose[0], ai_pose[1] - enemy_pose[1]
            dist = np.hypot(dx, dy)
            if dist > 0.01:
                ux, uy = dx/dist, dy/dist
            else:
                ux, uy = 1, 0
            
            # Point à safety_dist de l'ennemi (entre lui et nous)
            target_pos = (enemy_pose[0] + ux * self.safety_dist,
                          enemy_pose[1] + uy * self.safety_dist)
            
        # 2. Replanification (tous les 30 ticks ou si path vide)
        self.replan_timer += 1
        if self.replan_timer > 30 or not self.current_path:
            self.replan_timer = 0
            if self.planner:
                path = self.planner.plan(ai_pose[:2], target_pos)
                if path:
                    # Smoothing simple (downsample)
                    self.current_path = path[::2] if len(path) > 4 else path
                    if self.current_path[-1] != target_pos:
                        self.current_path.append(target_pos)
                    self.current_wp_idx = 0
                else:
                    # Fallback direct
                    self.current_path = [target_pos]
                    self.current_wp_idx = 0

        # 3. Suivi de chemin
        if self.current_path and self.current_wp_idx < len(self.current_path):
            wp = self.current_path[self.current_wp_idx]
            
            # Check waypoint reached
            dist_wp = np.hypot(ai_pose[0]-wp[0], ai_pose[1]-wp[1])
            if dist_wp < 0.05:
                self.current_wp_idx += 1
                if self.current_wp_idx >= len(self.current_path):
                    cmd['status'] = 'SUCCESS'
                    return cmd # Stop
            
            # Compute Control
            future_path = self.current_path[self.current_wp_idx:]
            v, w = self.controller.compute_control(ai_pose, future_path)
            cmd['v'] = v
            cmd['w'] = w
            
        cmd['debug_info'] = {'path': self.current_path, 'target': target_pos}
        return cmd

    def draw(self, surface, tm, debug_info):
        path = debug_info.get('path', [])
        if path and len(path) > 1:
            px_points = [tm.world_to_projector(p[0], p[1]) for p in path]
            pygame.draw.lines(surface, (0, 255, 100), False, px_points, 3)
            
        target = debug_info.get('target')
        if target:
            px = tm.world_to_projector(target[0], target[1])
            pygame.draw.circle(surface, (255, 255, 0), px, 8, 2)

if __name__ == '__main__':
    NavigationTask.run_standalone()
