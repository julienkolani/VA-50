"""
Arbre de Comportement - Version Avancée (basée sur bt_latest.py)

Fonctionnalités :
1. Structure hiérarchique : Selector, Sequence, Condition, Action
2. Stabilité Temporelle (Dwell Time) : Évite les oscillations d'état
3. Priorités : Survie > Attaque > Poursuite
"""

import time
import numpy as np
from enum import Enum
from typing import Dict, List, Optional

# --- CONFIG DEFAULTS ---
# Ces valeurs peuvent être surchargées par la config YAML
DEFAULT_BT_CONFIG = {
    "min_state_duration_s": 0.40,
    "min_state_duration_by_state_s": {
        "ATTAQUE (ALIGNEMENT)": 0.35,
        "ATTAQUE (VERROUILLÉ)": 0.25,
        "POURSUITE": 0.50,
        "RECHERCHE DE CIBLES": 0.20,
        "INITIALISATION": 0.10,
    },
    "force_switch_states": [
        "RETRAIT (MENACE DIRECTE)"
    ],
    "log_transitions": True
}

# --- PARAMÈTRES IA PAR DÉFAUT ---
SEUIL_MENACE_DEG = 15.0     # Angle max pour considérer que l'ennemi nous vise
SEUIL_LOCK_TIR_DEG = 5.0    # Précision requise pour l'état de tir

class BTStatus(Enum):
    SUCCESS = 0
    FAILURE = 1
    RUNNING = 2

class BTNode:
    def __init__(self, name):
        self.name = name

    def tick(self, ai_pose, en_pose, world) -> BTStatus:
        raise NotImplementedError

class SequenceNode(BTNode):
    def __init__(self, name, children: List[BTNode]):
        super().__init__(name)
        self.children = children

    def tick(self, ai_pose, en_pose, world) -> BTStatus:
        for child in self.children:
            status = child.tick(ai_pose, en_pose, world)
            if status != BTStatus.SUCCESS:
                return status
        return BTStatus.SUCCESS

class SelectorNode(BTNode):
    def __init__(self, name, children: List[BTNode]):
        super().__init__(name)
        self.children = children

    def tick(self, ai_pose, en_pose, world) -> BTStatus:
        for child in self.children:
            status = child.tick(ai_pose, en_pose, world)
            if status != BTStatus.FAILURE:
                return status
        return BTStatus.FAILURE

class ConditionNode(BTNode):
    def __init__(self, name, condition_func):
        super().__init__(name)
        self.condition_func = condition_func

    def tick(self, ai_pose, en_pose, world) -> BTStatus:
        if self.condition_func(ai_pose, en_pose, world):
            return BTStatus.SUCCESS
        return BTStatus.FAILURE

class ActionNode(BTNode):
    def __init__(self, name, action_func):
        super().__init__(name)
        self.action_func = action_func

    def tick(self, ai_pose, en_pose, world) -> BTStatus:
        self.action_func(ai_pose, en_pose, world)
        return BTStatus.SUCCESS


class TankBehaviorTree:
    """
    Arbre de comportement principal du Tank.
    Intègre la logique de décision et la stabilité temporelle.
    """
    def __init__(self, world_model, config: Optional[Dict] = None):
        self.world = world_model
        self.etat = "INITIALISATION"
        self.panic_until = 0.0 # Timer pour le mode panique
        
        # Output de l'arbre pour la stratégie
        self.current_decision = {
            'state': "INITIALISATION",
            'fire_request': False,
            'target_orientation': None,
            'has_los': False,
            'target_position': None # Sera rempli par la stratégie/tactique
        }

        # Config stabilité (dwell time)
        cfg = dict(DEFAULT_BT_CONFIG)
        if isinstance(config, dict):
            cfg.update(config)
            if "min_state_duration_by_state_s" in config:
                merged = dict(DEFAULT_BT_CONFIG.get("min_state_duration_by_state_s", {}))
                merged.update(config["min_state_duration_by_state_s"])
                cfg["min_state_duration_by_state_s"] = merged

        self.bt_cfg = cfg
        self._last_state_change_ts = time.time()
        self._build_tree()
        
    def _min_duration_for_state(self, state_name: str) -> float:
        by_state = self.bt_cfg.get("min_state_duration_by_state_s", {}) or {}
        if state_name in by_state:
            return float(by_state[state_name])
        return float(self.bt_cfg.get("min_state_duration_s", 0.4))

    def _can_switch(self, new_state: str) -> bool:
        now = time.time()
        dt = now - self._last_state_change_ts
        required = self._min_duration_for_state(self.etat)
        return dt >= required

    def _set_state(self, new_state: str, context_updates: Dict, reason: str = ""):
        """
        Met à jour l'état avec verrou temporel et applique les mises à jour de contexte (décisions).
        """
        # Toujours mettre à jour les intentions (tir, orientation) même si l'état ne change pas
        self.current_decision.update(context_updates)
        self.current_decision['state'] = self.etat # Default to current

        if new_state == self.etat:
            return

        force_states = set(self.bt_cfg.get("force_switch_states", []) or [])
        force = (new_state in force_states)

        if force or self._can_switch(new_state):
            prev = self.etat
            self.etat = new_state
            self.current_decision['state'] = new_state
            self._last_state_change_ts = time.time()
            
            if self.bt_cfg.get("log_transitions", True):
                extra = f" | {reason}" if reason else ""
                print(f"[BT] TRANSITION: {prev} -> {new_state}{extra}")

    # --- MÉTHODES UTILITAIRES / PRÉDICATS ---

    def check_ligne_de_vue(self, p1, p2):
        """Vérifie si le trajet est libre d'obstacles."""
        steps = 20
        for i in range(1, steps):
            tx = p1[0] + (p2[0] - p1[0]) * (i / steps)
            ty = p1[1] + (p2[1] - p1[1]) * (i / steps)
            if not self.world.is_position_valid(tx, ty):
                return False
        return True

    def est_vise_par_ennemi(self, ai_pose, en_pose):
        if ai_pose is None or en_pose is None:
            return False
            
        # Vecteur Ennemi -> IA
        dx, dy = ai_pose[0] - en_pose[0], ai_pose[1] - en_pose[1]
        angle_vers_ia = np.arctan2(dy, dx)
        
        # Angle de regard de l'ennemi
        theta_ennemi = en_pose[2]
        
        erreur_angle = (theta_ennemi - angle_vers_ia + np.pi) % (2 * np.pi) - np.pi
        erreur_deg = abs(np.degrees(erreur_angle))
        
        # Si l'ennemi regarde vers nous ET qu'il n'y a pas d'obstacle
        if erreur_deg < SEUIL_MENACE_DEG:
            return self.check_ligne_de_vue(en_pose[:2], ai_pose[:2])
        return False

    def _build_tree(self):
        """Construit la structure de l'arbre."""
        
        # --- ACTIONS ---
        
        def set_retrait(ai_pose, en_pose, world):
            self._set_state("RETRAIT (MENACE DIRECTE)", {
                'fire_request': False,
                'target_orientation': None # La stratégie gérera la fuite
            }, reason="Menace directe")

        def set_attaque_lock(ai_pose, en_pose, world):
            self._set_state("ATTAQUE (VERROUILLÉ)", {
                'fire_request': True,
                'target_orientation': en_pose[:2], # Vise l'ennemi
                'has_los': True
            }, reason="Tir possible")

        def set_attaque_align(ai_pose, en_pose, world):
            self._set_state("ATTAQUE (ALIGNEMENT)", {
                'fire_request': False, # Pas encore
                'target_orientation': en_pose[:2], # Vise l'ennemi
                'has_los': True
            }, reason="Alignement en cours")

        def set_poursuite(ai_pose, en_pose, world):
            self._set_state("POURSUITE", {
                'fire_request': False,
                'target_orientation': None, # La stratégie gérera le pathfinding
                'has_los': False
            }, reason="Recherche cible")

        # --- CONDITIONS ---

        def menace_directe(ai_pose, en_pose, world):
            is_threat = self.est_vise_par_ennemi(ai_pose, en_pose)
            
            # Logic Panic (Hysteresis)
            if is_threat:
                self.panic_until = time.time() + 2.0 # 2s de Panique assurée
            
            if time.time() < self.panic_until:
                return True
                
            return False

        def ligne_de_vue(ai_pose, en_pose, world):
            if ai_pose is None or en_pose is None:
                return False
            return self.check_ligne_de_vue(ai_pose[:2], en_pose[:2])

        def verrouille(ai_pose, en_pose, world):
            # Vérifie si NOUS visons l'ennemi
            if not ligne_de_vue(ai_pose, en_pose, world):
                return False
            dx, dy = en_pose[0] - ai_pose[0], en_pose[1] - ai_pose[1]
            angle_vers_en = np.arctan2(dy, dx)
            erreur_ia = (ai_pose[2] - angle_vers_en + np.pi) % (2 * np.pi) - np.pi
            return abs(np.degrees(erreur_ia)) < SEUIL_LOCK_TIR_DEG

        # --- ARBRE ---
        
        self.root = SelectorNode("Root", [
            # 1. RETRAIT (Survie)
            SequenceNode("Retrait", [
                ConditionNode("Menace ?", menace_directe),
                ActionNode("Action Retrait", set_retrait),
            ]),
            
            # 2. ATTAQUE (Aggressif)
            SelectorNode("Attaque", [
                # Tir si verrouillé
                SequenceNode("Tir", [
                    ConditionNode("LOS ?", ligne_de_vue),
                    ConditionNode("Lock ?", verrouille),
                    ActionNode("Action Tir", set_attaque_lock),
                ]),
                # Alignement si vu maiz pas lock
                SequenceNode("Align", [
                    ConditionNode("LOS ?", ligne_de_vue),
                    ActionNode("Action Align", set_attaque_align),
                ]),
            ]),
            
            # 3. POURSUITE (Défaut)
            ActionNode("Action Poursuite", set_poursuite),
        ])

    def execute(self, context) -> Dict:
        """
        Exécute l'arbre et retourne les décisions.
        Format du context attendu : { 'ai_pose', 'human_pose', ... }
        """
        ai_pose = context.get('ai_pose')
        en_pose = context.get('human_pose')
        world = self.world # Use internal world ref for obstacles

        if ai_pose is None or en_pose is None:
            self._set_state("RECHERCHE DE CIBLES", {
                'fire_request': False,
                'target_position': None
            }, reason="Perte visuelle")
            return self.current_decision

        self.root.tick(ai_pose, en_pose, world)
        return self.current_decision
