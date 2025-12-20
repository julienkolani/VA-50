"""
Arbre de Comportement - Framework de Décision IA

Implémente un arbre de comportement composable pour la prise de décision IA :
- Nœuds Sélecteurs (essaient les enfants jusqu'à succès)
- Nœuds Séquences (exécutent tous les enfants dans l'ordre)
- Nœuds Conditions (vérifient l'état du monde)
- Nœuds Actions (produisent des décisions)

L'IA ne modifie PAS directement l'état du jeu.
Elle retourne seulement des INTENTIONS : (position_cible, demande_tir).

Logs : [BT] Nœud X succès/échec
"""

from enum import Enum
from abc import ABC, abstractmethod
from .decisions import (
    is_enemy_too_close,
    has_line_of_sight,
    is_optimal_firing_range,
    should_retreat,
    find_nearest_cover,
    calculate_flank_position
)


class NodeStatus(Enum):
    """Statut d'exécution du nœud de l'arbre comportemental."""
    SUCCESS = "success"
    FAILURE = "failure"
    RUNNING = "running"


class BTNode(ABC):
    """
    Classe de base pour tous les nœuds de l'arbre comportemental.
    
    Tous les nœuds implémentent tick() qui retourne un NodeStatus.
    """
    
    def __init__(self, name):
        self.name = name
    
    @abstractmethod
    def tick(self, context):
        """
        Exécute ce nœud avec le contexte donné.
        
        Args:
            context: dict avec état du monde, poses robots, etc.
            
        Returns:
            NodeStatus
        """
        pass


class Selector(BTNode):
    """
    Nœud Sélecteur : essaie les enfants jusqu'à ce que l'un réussisse.
    
    Retourne SUCCESS si un enfant réussit.
    Retourne FAILURE si tous les enfants échouent.
    """
    
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children
    
    def tick(self, context):
        for child in self.children:
            status = child.tick(context)
            if status == NodeStatus.SUCCESS:
                return NodeStatus.SUCCESS
            elif status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
        return NodeStatus.FAILURE


class Sequence(BTNode):
    """
    Nœud Séquence : exécute les enfants dans l'ordre.
    
    Retourne SUCCESS si tous les enfants réussissent.
    Retourne FAILURE si un enfant échoue.
    """
    
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children
    
    def tick(self, context):
        for child in self.children:
            status = child.tick(context)
            if status == NodeStatus.FAILURE:
                return NodeStatus.FAILURE
            elif status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
        return NodeStatus.SUCCESS


class Condition(BTNode):
    """
    Nœud Condition : vérifie une fonction prédicat.
    
    Retourne SUCCESS si le prédicat est Vrai, FAILURE sinon.
    """
    
    def __init__(self, name, predicate_fn):
        super().__init__(name)
        self.predicate_fn = predicate_fn
    
    def tick(self, context):
        if self.predicate_fn(context):
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE


class Action(BTNode):
    """
    Nœud Action : exécute une fonction d'action.
    
    La fonction d'action modifie context['ai_output'] avec les décisions.
    """
    
    def __init__(self, name, action_fn):
        super().__init__(name)
        self.action_fn = action_fn
    
    def tick(self, context):
        return self.action_fn(context)


# --- Action Functions ---

def action_retreat_to_cover(context):
    """
    Action : Trouver une couverture et définir comme cible.
    
    Modifie context['ai_output'] avec la cible de repli.
    """
    cover_pos = find_nearest_cover(context)
    
    if cover_pos is not None:
        context['ai_output']['target_position'] = cover_pos
        context['ai_output']['state'] = 'RETREAT'
        context['ai_output']['fire_request'] = False
        print("[BT] Action : REPLI vers couverture à ({:.2f}, {:.2f})".format(
            cover_pos[0], cover_pos[1]))
        return NodeStatus.SUCCESS
    else:
        print("[BT] Action : REPLI échoué - pas de couverture")
        return NodeStatus.FAILURE


def action_aim_and_fire(context):
    """
    Action : Viser l'ennemi et demander le tir.
    
    Modifie context['ai_output'] avec la demande de tir.
    """
    human_pose = context.get('human_pose')
    
    if human_pose is None:
        return NodeStatus.FAILURE
    
    context['ai_output']['target_position'] = None  # Rester sur place
    context['ai_output']['target_orientation'] = human_pose[:2]  # Viser l'ennemi
    context['ai_output']['fire_request'] = True
    context['ai_output']['state'] = 'ATTACK'
    
    print("[BT] Action : VISER ET TIRER sur ennemi")
    return NodeStatus.SUCCESS


def action_find_flank_position(context):
    """
    Action : Calculer une position de contournement.
    """
    flank_pos = calculate_flank_position(context)
    
    if flank_pos is not None:
        context['ai_output']['target_position'] = flank_pos
        context['ai_output']['state'] = 'FLANK'
        context['ai_output']['fire_request'] = False
        print("[BT] Action : CONTOURNEMENT vers ({:.2f}, {:.2f})".format(
            flank_pos[0], flank_pos[1]))
        return NodeStatus.SUCCESS
    else:
        return NodeStatus.FAILURE


def action_move_to_flank(context):
    """
    Action : Se déplacer vers la position de contournement.
    """
    # C'est géré par le suiveur de trajectoire, confirmer juste qu'on a une cible
    target = context['ai_output'].get('target_position')
    if target is not None:
        print("[BT] Action : Déplacement vers position contournement")
        return NodeStatus.RUNNING
    return NodeStatus.FAILURE


def action_hunt_enemy(context):
    """
    Action : Se déplacer vers la dernière position connue de l'ennemi.
    """
    human_pose = context.get('human_pose')
    
    if human_pose is not None:
        context['ai_output']['target_position'] = human_pose[:2]
        context['ai_output']['state'] = 'HUNT'
        context['ai_output']['fire_request'] = False
        print("[BT] Action : CHASSE - déplacement vers position ennemie")
        return NodeStatus.SUCCESS
    
    return NodeStatus.FAILURE


def build_ai_behavior_tree():
    """
    Construit l'arbre de comportement principal de l'IA.
    
    Structure :
    
    Sélecteur (Racine)
      +-- Séquence (SURVIVAL)
      |   +-- Condition : "ennemi trop proche ?"
      |   +-- Action : "repli vers couverture"
      |
      +-- Séquence (ATTACK)
      |   +-- Condition : "ligne de vue ?"
      |   +-- Condition : "portée optimale ?"
      |   +-- Action : "viser et demander feu"
      |
      +-- Séquence (FLANK)
          +-- Action : "trouver position contournement"
          +-- Action : "déplacement contournement"
    
    Returns:
        BTNode: racine de l'arbre comportemental
    """
    # Branche SURVIVAL : repli si ennemi trop proche
    survival_sequence = Sequence("SURVIVAL", [
        Condition("enemy_too_close", is_enemy_too_close),
        Action("retreat_to_cover", action_retreat_to_cover)
    ])
    
    # Branche ATTACK : tir si tir clair et portée optimale
    attack_sequence = Sequence("ATTACK", [
        Condition("has_line_of_sight", has_line_of_sight),
        Condition("in_optimal_range", is_optimal_firing_range),
        Action("aim_and_fire", action_aim_and_fire)
    ])
    
    # Branche FLANK : essayer d'obtenir une meilleure position
    flank_sequence = Sequence("FLANK", [
        Action("find_flank_position", action_find_flank_position),
        Action("move_to_flank", action_move_to_flank)
    ])
    
    # Branche HUNT : repli - juste chasser l'ennemi
    hunt_action = Action("hunt_enemy", action_hunt_enemy)
    
    # Sélecteur racine : essaie survie d'abord, puis attaque, puis contournement, puis chasse
    root = Selector("AI_ROOT", [
        survival_sequence,
        attack_sequence,
        flank_sequence,
        hunt_action
    ])
    
    print("[BT] Arbre de comportement construit")
    return root


class BehaviorTreeExecutor:
    """
    Exécuteur qui lance l'arbre comportemental à chaque tick.
    """
    
    def __init__(self):
        self.tree = build_ai_behavior_tree()
    
    def execute(self, context):
        """
        Exécute l'arbre comportemental avec le contexte donné.
        
        Args:
            context: Dictionnaire d'état du monde. Doit contenir :
                - ai_pose: (x, y, theta)
                - human_pose: (x, y, theta)
                - occupancy_grid: objet grille
                
        Returns:
            dict: Décisions de sortie IA
        """
        # Initialise la structure de sortie
        context['ai_output'] = {
            'target_position': None,
            'target_orientation': None,
            'fire_request': False,
            'state': 'IDLE',
            'has_los': False
        }
        
        # Vérifie LOS pour la sortie
        context['ai_output']['has_los'] = has_line_of_sight(context)
        
        # Exécute l'arbre
        status = self.tree.tick(context)
        
        print("[BT] Exécution arbre : {}".format(status.value))
        
        return context['ai_output']
