"""
Stratégie - Orchestrateur IA (Architecture Task-Based)

Rôle :
1. Interroge le Cerveau (Behavior Tree) pour connaître l'Intention (RETRAIT, ATTAQUE, POURSUITE).
2. Active la Tâche correspondante (FleeTask, AttackTask, NavigationTask).
3. Délègue l'exécution à la tâche active.
"""

from typing import Dict, Optional
from .behavior_tree import TankBehaviorTree
from core.tasks.navigation import NavigationTask
from core.tasks.survival import FleeTask
from core.tasks.combat import AttackTask
from core.ia.tactical import TacticalAnalyzer

class AIStrategy:
    def __init__(self, config):
        self.config = config
        self.brain = None
        self.world_model = None
        
        # Gestion des Tâches
        self.tasks = {
            'NAV': NavigationTask(target_type='ENEMY'),
            'FLEE': FleeTask(),
            'ATTACK': AttackTask()
        }
        self.current_task = self.tasks['NAV'] # Par défaut
        self.current_state = "INITIALISATION"
        
        # Pour visualisation
        self.current_path = [] 
        
        print("[AI] Stratégie v3 (Task-Based) : Initialisée")

    def set_planner(self, grid):
        pass # Obsolète mais gardé pour compatibilité

    def set_world_model(self, world_model):
        self.world_model = world_model
        # Le BT a besoin du world model
        bt_conf = self.config.get('behavior_tree', {})
        self.brain = TankBehaviorTree(world_model, config=bt_conf)
        
        # Certaines tâches ont besoin d'init (ex: TacticalAnalyzer pour Flee)
        # Mais elles le font en lazy loading dans execute(), donc c'est ok.

    def decide(self, world_state: Dict) -> Dict:
        # Sortie par défaut
        decision = {
            'state': "IDLE", 'fire_request': False, 
            'target_position': None, 'target_orientation': None, 'has_los': False,
            'v': 0.0, 'w': 0.0 # Nouvelles sorties directes !
        }

        ai_pose = world_state.get('ai_pose')
        enemy_pose = world_state.get('human_pose')
        
        if not ai_pose or not enemy_pose or not self.brain:
            return decision

        # 1. Cerveau : QUOI FAIRE ?
        context = {'ai_pose': ai_pose, 'human_pose': enemy_pose}
        bt_output = self.brain.execute(context)
        new_state = bt_output['state']
        
        # 2. Switching de Tâche
        # Mapping État BT -> Tâche
        if "RETRAIT" in new_state:
            self._switch_task('FLEE')
        elif "ATTAQUE" in new_state:
            # Subtilité : "ATTAQUE (ALIGNEMENT)" vs "ATTAQUE (VERROUILLÉ)"
            # Les deux sont gérés par AttackTask (qui vise puis tire)
            self._switch_task('ATTACK')
        elif "POURSUITE" in new_state:
            self._switch_task('NAV')
        else:
             # IDLE ou RECHERCHE -> Navigation par défaut (ou une tache SearchTask future)
             self._switch_task('NAV')
             
        # 3. Exécution Tâche : COMMENT LE FAIRE ?
        # On enrichit le contexte avec world et dt (approximatif ici, 
        # idéalement passé par run_game, mais BaseTask gère sans dt critique pour l'instant)
        task_context = {
            'ai_pose': ai_pose,
            'human_pose': enemy_pose,
            'world': self.world_model,
            'dt': 0.033 # Dummy dt, utilisé par Kalman interne si besoin
        }
        
        cmd = self.current_task.execute(task_context)
        
        # 4. Packaging Sortie
        decision['state'] = new_state
        decision['v'] = cmd.get('v', 0.0)
        decision['w'] = cmd.get('w', 0.0)
        decision['fire_request'] = cmd.get('fire_request', False)
        
        # Info debug pour le renderer
        debug = cmd.get('debug_info', {})
        self.current_path = debug.get('path', []) # Pour affichage chemin
        decision['target_position'] = debug.get('target') # Pour affichage cible
        decision['debug_info'] = debug # NEW: Propagation complète pour le debugger
        
        return decision

    def _switch_task(self, task_key):
        if self.tasks[task_key] != self.current_task:
            print(f"[AI] Switch Task : {self.current_task.name} -> {self.tasks[task_key].name}")
            self.current_task = self.tasks[task_key]
            # On pourrait appeler self.current_task.reset() ici

    # Interface Legacy pour run_game.py (affichage chemin)
    def get_full_path(self):
        return self.current_path
        
    def advance_waypoint(self):
        # Plus nécessaire car géré par NavigationTask en interne
        pass
