"""
Stratégie - Contrôleur IA Haut-Niveau

Orchestre le système IA :
1. Lit l'état du monde
2. Exécute l'arbre de comportement
3. Déclenche la planification de chemin si nécessaire
4. Retourne les décisions IA (cible, demande_tir)

Ceci est le point d'entrée principal pour le sous-système IA.

Logs : [AI] État : ATTACK/FLANK/RETREAT, cible=(x,y), tir=True/False
"""

import numpy as np
from typing import Dict, Tuple, Optional
from .behavior_tree import BehaviorTreeExecutor, NodeStatus
from .decisions import has_line_of_sight, is_enemy_too_close, is_optimal_firing_range
from .planners.a_star import AStarPlanner


class AIStrategy:
    """
    Contrôleur IA principal.
    
    Combine arbre de comportement + planification de chemin pour produire les actions IA.
    """
    
    def __init__(self, config):
        """
        Initialise la stratégie IA.
        
        Args:
            config: Configuration IA depuis config/ia.yaml
        """
        self.config = config
        self.behavior_tree = BehaviorTreeExecutor()
        self.planner = None  # Set when world is available
        
        # État IA
        self.current_path = []
        self.current_waypoint_idx = 0
        self.state = "IDLE"  # ATTACK, FLANK, RETREAT, HUNT
        
        # Contrôle du taux de décision
        self.decision_interval = config.get('decision_rate', {}).get('replan_interval', 10)
        self.tick_count = 0
        
        print("[AI] Stratégie initialisée")
        
    def set_planner(self, occupancy_grid):
        """
        Initialise le planificateur de chemin avec la grille d'occupation.
        
        Args:
            occupancy_grid: OccupancyGrid depuis core/world
        """
        heuristic = self.config.get('strategy', {}).get('heuristic', 'euclidean')
        self.planner = AStarPlanner(occupancy_grid, heuristic)
        print("[AI] Planificateur chemin initialisé avec heuristique {}".format(heuristic))
        
    def decide(self, world_state: Dict) -> Dict:
        """
        Fonction de décision principale appelée à chaque tick.
        
        Args:
            world_state: {
                'ai_pose': (x, y, theta),
                'human_pose': (x, y, theta),
                'occupancy_grid': objet grille,
                'raycast_sys': objet raycast,
                'game_time': float,
            }
            
        Returns:
            {
                'target_position': (x, y) ou None,
                'target_orientation': theta ou None,
                'fire_request': bool,
                'state': str (ATTACK/FLANK/RETREAT),
                'has_los': bool,
            }
        """
        self.tick_count += 1
        
        ai_pose = world_state.get('ai_pose')
        enemy_pose = world_state.get('human_pose')
        
        # Sortie par défaut
        decision = {
            'target_position': None,
            'target_orientation': None,
            'fire_request': False,
            'state': self.state,
            'has_los': False
        }
        
        if ai_pose is None or enemy_pose is None:
            return decision

        # Prépare le contexte pour l'arbre de comportement
        context = {
            'ai_pose': ai_pose,
            'human_pose': enemy_pose,
            'occupancy_grid': world_state.get('occupancy_grid'),
            'raycast_sys': world_state.get('raycast_sys'),
        }
        
        # Exécute l'arbre de comportement
        bt_output = self.behavior_tree.execute(context)
        
        # Copie les décisions de l'arbre comportemental
        decision['fire_request'] = bt_output.get('fire_request', False)
        decision['has_los'] = bt_output.get('has_los', False)
        decision['state'] = bt_output.get('state', 'IDLE')
        decision['target_orientation'] = bt_output.get('target_orientation')
        self.state = decision['state']
        
        target_pos = bt_output.get('target_position')
        
        # Planification de Chemin - replanifie seulement périodiquement ou si la cible change significativement
        if target_pos is not None and self.planner:
            should_replan = False
            
            # Replanifie si pas de chemin ou sur intervalle
            if not self.current_path:
                should_replan = True
            elif self.tick_count % self.decision_interval == 0:
                should_replan = True
            elif len(self.current_path) > 0:
                # Replanifie si la cible a bougé significativement
                last_goal = self.current_path[-1]
                dist_to_new = np.linalg.norm(
                    np.array(last_goal) - np.array(target_pos)
                )
                if dist_to_new > 0.5:
                    should_replan = True
            
            if should_replan:
                path = self.planner.plan(ai_pose[:2], target_pos)
                if path:
                    self.current_path = path
                    self.current_waypoint_idx = 0
                    print("[AI] Nouveau chemin planifié : {} waypoints".format(len(path)))
                else:
                    # FALLBACK: Si A* échoue, force un waypoint direct vers la cible
                    # Cela évite que le robot reste immobile
                    print("[AI] ATTENTION: Pas de chemin trouvé, waypoint direct forcé")
                    self.current_path = [target_pos]
                    self.current_waypoint_idx = 0
        
        decision['target_position'] = target_pos
        
        # Log décision
        if self.tick_count % 30 == 0:  # Log chaque seconde à 30 FPS
            self._log_decision(decision)
        
        return decision
    
    def _log_decision(self, decision):
        """Log la décision IA actuelle."""
        target = decision.get('target_position')
        target_str = "({:.2f}, {:.2f})".format(target[0], target[1]) if target else "None"
        
        print("[AI] État : {}, cible={}, tir={}, LOS={}".format(
            decision['state'],
            target_str,
            decision['fire_request'],
            decision['has_los']
        ))
    
    def get_next_waypoint(self) -> Optional[Tuple[float, float]]:
        """
        Obtient le prochain waypoint du chemin actuel.
        
        Utilisé par le module de contrôle pour le suivi de trajectoire.
        
        Returns:
            (x, y) du prochain waypoint, ou None si pas de chemin
        """
        if self.current_waypoint_idx >= len(self.current_path):
            return None
        return self.current_path[self.current_waypoint_idx]
    
    def advance_waypoint(self):
        """
        Marque le waypoint actuel comme atteint, passe au suivant.
        
        Appelé par le module de contrôle quand le waypoint est atteint.
        """
        self.current_waypoint_idx += 1
        
    def get_full_path(self) -> list:
        """
        Obtient le chemin complet actuel pour la visualisation.
        
        Returns:
            Liste de waypoints (x, y)
        """
        return self.current_path
