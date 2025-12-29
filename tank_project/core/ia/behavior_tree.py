"""
Arbre de Comportement - Stratégie Chasseur Simple

Implémente une stratégie simplifiée "Chasseur" avec priorité stricte :
1. TIRER : Si Ennemi Visible ET Aligné -> Stop & Feu
2. VISER : Si Ennemi Visible MAIS Non Aligné -> Tourner sur place
3. CHASSER : Si Ennemi Non Visible -> Aller vers dernière position connue

Logs : [BT] Action ...
"""

from enum import Enum
from abc import ABC, abstractmethod
from typing import Dict, Optional

# Import des prédicats de décision
from .decisions import (
    has_line_of_sight,
    is_aimed_at_enemy
)


class NodeStatus(Enum):
    """Statut d'exécution du nœud de l'arbre comportemental."""
    SUCCESS = "success"
    FAILURE = "failure"
    RUNNING = "running"


class BTNode(ABC):
    def __init__(self, name):
        self.name = name
    
    @abstractmethod
    def tick(self, context):
        pass


class Selector(BTNode):
    """Essaie les enfants jusqu'à succès."""
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
    """Exécute tous les enfants dans l'ordre."""
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
    """Vérifie un prédicat."""
    def __init__(self, name, predicate_fn):
        super().__init__(name)
        self.predicate_fn = predicate_fn
    
    def tick(self, context):
        if self.predicate_fn(context):
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE


class Action(BTNode):
    """Exécute une action."""
    def __init__(self, name, action_fn):
        super().__init__(name)
        self.action_fn = action_fn
    
    def tick(self, context):
        return self.action_fn(context)


# --- Fonctions d'Action Simplifiées ---

def action_shoot(context):
    """Action: TIRER (Stop mouvement + Feu)"""
    human_pose = context.get('human_pose')
    if human_pose is None: return NodeStatus.FAILURE
    
    # On définit la cible de tir
    context['ai_output']['target_position'] = None # Stop
    context['ai_output']['target_orientation'] = human_pose[:2] # Viser coord
    context['ai_output']['fire_request'] = True
    context['ai_output']['state'] = 'ATTACK'
    
    print("[BT] Action : FEU ! (Aligné et Visible)")
    return NodeStatus.SUCCESS

def action_aim(context):
    """Action: VISER (Tourner vers ennemi, sans tirer)"""
    human_pose = context.get('human_pose')
    if human_pose is None: return NodeStatus.FAILURE
    
    context['ai_output']['target_position'] = None # Stop pour tourner
    context['ai_output']['target_orientation'] = human_pose[:2]
    context['ai_output']['fire_request'] = False # Pas encore
    context['ai_output']['state'] = 'AIMING'
    
    print("[BT] Action : VISER (Rotation vers ennemi)")
    return NodeStatus.SUCCESS

def action_hunt(context):
    """Action: CHASSER (Aller vers dernière position connue)"""
    human_pose = context.get('human_pose')
    if human_pose is not None:
        context['ai_output']['target_position'] = human_pose[:2]
        context['ai_output']['fire_request'] = False
        context['ai_output']['state'] = 'HUNT'
        print("[BT] Action : CHASSER (Déplacement vers cible)")
        return NodeStatus.SUCCESS
    
    # Si on ne connait pas la position ennemie (jamais vu?), on reste sur place ou on patrouille
    # Pour l'instant on reste IDLE
    context['ai_output']['state'] = 'IDLE'
    print("[BT] Action : IDLE (Ennemi inconnu)")
    return NodeStatus.FAILURE


def build_ai_behavior_tree():
    """
    Construit l'arbre Chasseur Simple.
    
    Racine (Selector)
      1. Sequence (TIRER)
         - Condition: LOS ?
         - Condition: Aligné ?
         - Action: FEU
      2. Sequence (VISER)
         - Condition: LOS ?
         - Action: VISER
      3. Action (CHASSER)
    """
    
    # 1. TIRER
    seq_shoot = Sequence("SHOOT_SEQ", [
        Condition("HasLOS", has_line_of_sight),
        Condition("IsAligned", is_aimed_at_enemy),
        Action("Shoot", action_shoot)
    ])
    
    # 2. VISER
    seq_aim = Sequence("AIM_SEQ", [
        Condition("HasLOS", has_line_of_sight),
        Action("Aim", action_aim)
    ])
    
    # 3. CHASSER
    act_hunt = Action("Hunt", action_hunt)
    
    return Selector("ROOT_HUNTER", [
        seq_shoot,
        seq_aim,
        act_hunt
    ])


class BehaviorTreeExecutor:
    def __init__(self):
        self.tree = build_ai_behavior_tree()
    
    def execute(self, context):
        # Init structure sortie
        context['ai_output'] = {
            'target_position': None,
            'target_orientation': None,
            'fire_request': False,
            'state': 'IDLE',
            'has_los': False
        }
        
        # Helper LOS pour info debug
        context['ai_output']['has_los'] = has_line_of_sight(context)
        
        status = self.tree.tick(context)
        # print(f"[BT] Tick Status: {status.value}")
        
        return context['ai_output']
