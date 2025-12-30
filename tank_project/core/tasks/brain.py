from core.tasks.base_task import BaseTask
from core.ia.strategy import AIStrategy
import pygame
import yaml
from pathlib import Path

class BrainTask(BaseTask):
    """
    Tâche Maître "Cerveau" : Exécute toute la pile IA (Vision -> BT -> Tâche).
    Permet de visualiser l'état mental du robot et la tâche active.
    """
    def __init__(self):
        super().__init__("MasterBrain")
        
        # Chargement Config IA
        root_dir = Path(__file__).parent.parent.parent
        config_path = root_dir / 'config' / 'ia.yaml'
        
        ia_config = {}
        if config_path.exists():
            with open(config_path) as f:
                ia_config = yaml.safe_load(f)
        
        # Instanciation Stratégie
        self.strategy = AIStrategy(ia_config)
        self.last_decision = {}
        
    def execute(self, context):
        """Délegue tout à la stratégie."""
        
        # Init Late (besoin du world model)
        if self.strategy.world_model is None:
            self.strategy.set_world_model(context['world'])
            
        # Appel Stratégie
        # Note: 'context' de BaseTask est compatible avec 'world_state' de Strategy
        decision = self.strategy.decide(context)
        self.last_decision = decision
        
        # Mapping Sortie Strategy -> IHM BaseTask
        cmd = {
            'v': decision.get('v', 0.0),
            'w': decision.get('w', 0.0),
            'fire_request': decision.get('fire_request', False),
            'status': decision.get('state', 'UNKNOWN'),
            # On passe tout le decision pack comme debug info pour le draw
            'debug_info': decision 
        }
        return cmd

    def draw(self, surface, tm, debug_info):
        # 1. Dessin de la Sous-Tâche Active
        # On récupère la tâche active via la stratégie
        current_task = self.strategy.current_task
        
        # On extrait les infos debug spécifiques à la sous-tâche
        sub_debug_info = debug_info.get('debug_info', {}) 
        
        if current_task:
            current_task.draw(surface, tm, sub_debug_info)
            
        # 2. Overlay "Cerveau" (État, Intention)
        font_big = pygame.font.SysFont("Impact", 30)
        font_small = pygame.font.SysFont("Consolas", 18)
        
        state = debug_info.get('state', 'N/A')
        task_name = current_task.name if current_task else "None"
        
        # Couleur selon état
        color = (200, 200, 200)
        if "ATTAQUE" in state: color = (255, 50, 50)
        elif "RETRAIT" in state: color = (50, 255, 50)
        elif "POURSUITE" in state: color = (50, 100, 255)
        
        # Rendu Texte
        y = 50
        lines = [
            (f"BRAIN: {state}", color, font_big),
            (f"TASK:  {task_name}", (255, 255, 255), font_small),
        ]
        
        for text, col, fnt in lines:
            s = fnt.render(text, True, col)
            surface.blit(s, (10, y))
            y += s.get_height() + 5

if __name__ == '__main__':
    BrainTask.run_standalone()
