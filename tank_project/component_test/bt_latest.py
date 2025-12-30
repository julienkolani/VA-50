#!/usr/bin/env python3
import sys
import os
import yaml
import pygame
import numpy as np
import cv2
import time
from pathlib import Path
from enum import Enum

# Ajout du chemin racine pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.camera.realsense_stream import RealSenseStream
from perception.camera.aruco_detector import ArucoDetector
from core.world.world_model import WorldModel
from core.world.unified_transform import load_calibration

# --- CONFIGURATION VISUELLE ---
C_BG = (5, 5, 15)
C_RETRAIT = (255, 50, 50)   # Rouge
C_ATTAQUE = (0, 255, 100)   # Vert
C_POURSUITE = (0, 150, 255) # Bleu
C_CYAN = (0, 255, 255)      # IA
C_WHITE = (255, 255, 255)

# --- PARAMÈTRES IA (par défaut, surchargeables via YAML) ---
ROBOT_AI_ID = 5
ROBOT_ENEMY_ID = 4
SEUIL_MENACE_DEG = 15.0     # Angle max pour considérer que l'ennemi nous vise
SEUIL_LOCK_TIR_DEG = 5.0    # Précision requise pour l'état de tir

# --- CONFIG "STABILITÉ D'ÉTATS" (NOUVEAU) ---
# But: empêcher les oscillations rapides d'état (debounce/dwell time)
DEFAULT_BT_CONFIG = {
    "min_state_duration_s": 0.40,  # durée minimale avant d'autoriser un changement d'état
    "min_state_duration_by_state_s": {
        # Durées par état (optionnel) : surcharge le global si présent
        "ATTAQUE (ALIGNEMENT)": 0.35,
        "ATTAQUE (VERROUILLÉ)": 0.25,
        "POURSUITE": 0.50,
        "RECHERCHE DE CIBLES": 0.20,
        "INITIALISATION": 0.10,
    },
    # Les états critiques peuvent casser le verrou temporel
    "force_switch_states": [
        "RETRAIT (MENACE DIRECTE)"
    ],
    # Logger les transitions
    "log_transitions": True
}

# --- NOEUDS DE L'ARBRE DE COMPORTEMENT ---
class BTStatus(Enum):
    SUCCESS = 0
    FAILURE = 1
    RUNNING = 2

class BTNode:
    def __init__(self, name):
        self.name = name

    def tick(self, ai_pose, en_pose, world):
        raise NotImplementedError

class SequenceNode(BTNode):
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children

    def tick(self, ai_pose, en_pose, world):
        for child in self.children:
            status = child.tick(ai_pose, en_pose, world)
            if status != BTStatus.SUCCESS:
                return status
        return BTStatus.SUCCESS

class SelectorNode(BTNode):
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children

    def tick(self, ai_pose, en_pose, world):
        for child in self.children:
            status = child.tick(ai_pose, en_pose, world)
            if status != BTStatus.FAILURE:
                return status
        return BTStatus.FAILURE

class ConditionNode(BTNode):
    def __init__(self, name, condition_func):
        super().__init__(name)
        self.condition_func = condition_func

    def tick(self, ai_pose, en_pose, world):
        if self.condition_func(ai_pose, en_pose, world):
            return BTStatus.SUCCESS
        return BTStatus.FAILURE

class ActionNode(BTNode):
    def __init__(self, name, action_func):
        super().__init__(name)
        self.action_func = action_func

    def tick(self, ai_pose, en_pose, world):
        self.action_func(ai_pose, en_pose, world)
        return BTStatus.SUCCESS

# --- ARBRE DE COMPORTEMENT ---
class TankBehaviorTree:
    def __init__(self, world_model, config=None):
        self.world = world_model
        self.etat = "INITIALISATION"

        # Config stabilité (dwell time)
        cfg = dict(DEFAULT_BT_CONFIG)
        if isinstance(config, dict):
            # merge shallow + merge dictionnaire nested si fourni
            cfg.update(config)
            if "min_state_duration_by_state_s" in config:
                merged = dict(DEFAULT_BT_CONFIG.get("min_state_duration_by_state_s", {}))
                merged.update(config["min_state_duration_by_state_s"])
                cfg["min_state_duration_by_state_s"] = merged

        self.bt_cfg = cfg
        self._last_state_change_ts = time.time()
        self._pending_state = None  # optionnel (si tu veux tracer)
        self._build_tree()

    # --- utilitaires stabilité d'état ---
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

    def _set_state(self, new_state: str, reason: str = ""):
        """
        Setter unique d'état avec verrou temporel.
        - Force le switch si état critique (force_switch_states)
        - Sinon, respecte min_state_duration (debounce)
        """
        if new_state == self.etat:
            return

        force_states = set(self.bt_cfg.get("force_switch_states", []) or [])
        force = (new_state in force_states)

        if force or self._can_switch(new_state):
            prev = self.etat
            self.etat = new_state
            self._last_state_change_ts = time.time()
            if self.bt_cfg.get("log_transitions", True):
                extra = f" | {reason}" if reason else ""
                print(f"[BT] TRANSITION: {prev} -> {new_state}{extra}")
        else:
            # On refuse le changement pour éviter les oscillations
            self._pending_state = new_state
            # Log léger et non-spam (optionnel)
            # print(f"[BT] HOLD: {self.etat} (pending {new_state})")

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
        """Détermine si l'ennemi pointe son canon vers l'IA avec une ligne de vue claire."""
        if ai_pose is None or en_pose is None:
            return False
        dx, dy = ai_pose[0] - en_pose[0], ai_pose[1] - en_pose[1]
        angle_vers_ia = np.arctan2(dy, dx)
        erreur_angle = (en_pose[2] - angle_vers_ia + np.pi) % (2 * np.pi) - np.pi
        erreur_deg = abs(np.degrees(erreur_angle))
        if erreur_deg < SEUIL_MENACE_DEG:
            return self.check_ligne_de_vue(en_pose[:2], ai_pose[:2])
        return False

    def _build_tree(self):
        """Construit l'arbre de comportement."""
        # Actions (utilisent _set_state pour éviter les changements trop rapides)
        def set_retrait(ai_pose, en_pose, world):
            self._set_state("RETRAIT (MENACE DIRECTE)", reason="Menace directe")

        def set_attaque_lock(ai_pose, en_pose, world):
            self._set_state("ATTAQUE (VERROUILLÉ)", reason="LOS + verrouillage")

        def set_attaque_align(ai_pose, en_pose, world):
            self._set_state("ATTAQUE (ALIGNEMENT)", reason="LOS sans verrouillage")

        def set_poursuite(ai_pose, en_pose, world):
            self._set_state("POURSUITE", reason="Ennemi hors LOS")

        # Conditions
        def menace_directe(ai_pose, en_pose, world):
            return self.est_vise_par_ennemi(ai_pose, en_pose)

        def ligne_de_vue(ai_pose, en_pose, world):
            if ai_pose is None or en_pose is None:
                return False
            return self.check_ligne_de_vue(ai_pose[:2], en_pose[:2])

        def verrouille(ai_pose, en_pose, world):
            if not ligne_de_vue(ai_pose, en_pose, world):
                return False
            dx, dy = en_pose[0] - ai_pose[0], en_pose[1] - ai_pose[1]
            angle_vers_en = np.arctan2(dy, dx)
            erreur_ia = (ai_pose[2] - angle_vers_en + np.pi) % (2 * np.pi) - np.pi
            return abs(np.degrees(erreur_ia)) < SEUIL_LOCK_TIR_DEG

        # Construction de l'arbre
        self.root = SelectorNode("Root", [
            # Priorité 1 : Retrait si menace directe
            SequenceNode("Retrait", [
                ConditionNode("Menace directe ?", menace_directe),
                ActionNode("Passer en retrait", set_retrait),
            ]),
            # Priorité 2 : Attaque si ligne de vue
            SelectorNode("Attaque", [
                SequenceNode("Attaque verrouillée", [
                    ConditionNode("Ligne de vue ?", ligne_de_vue),
                    ConditionNode("Verrouillé ?", verrouille),
                    ActionNode("Attaque verrouillée", set_attaque_lock),
                ]),
                SequenceNode("Attaque alignement", [
                    ConditionNode("Ligne de vue ?", ligne_de_vue),
                    ActionNode("Attaque alignement", set_attaque_align),
                ]),
            ]),
            # Priorité 3 : Poursuite par défaut
            ActionNode("Poursuite", set_poursuite),
        ])

    def update(self, ai_pose, en_pose):
        """Met à jour l'état en exécutant l'arbre de comportement (avec dwell time)."""
        if ai_pose is None or en_pose is None:
            # Ici aussi on passe par _set_state pour éviter oscillation "SEARCH <-> POURSUITE"
            self._set_state("RECHERCHE DE CIBLES", reason="Pose manquante")
            return self.etat

        self.root.tick(ai_pose, en_pose, self.world)
        return self.etat

# --- FONCTIONS AUXILIAIRES ---
def get_robot_pose_in_world(detection_data, transform_mgr):
    u, v = detection_data['center']
    theta_pix = detection_data['orientation']
    u_f = u + 20 * np.cos(theta_pix)
    v_f = v + 20 * np.sin(theta_pix)
    x, y = transform_mgr.camera_to_world(u, v)
    xf, yf = transform_mgr.camera_to_world(u_f, v_f)
    x = np.clip(x, 0.01, transform_mgr.arena_width_m - 0.01)
    y = np.clip(y, 0.01, transform_mgr.arena_height_m - 0.01)
    return x, y, np.arctan2(yf - y, xf - x)

def _load_bt_config(config_dir: Path) -> dict:
    """
    Charge une config IA optionnelle depuis config/ia_bt.yaml (ou renvoie defaults).
    Format attendu (exemple):
      bt:
        min_state_duration_s: 0.4
        min_state_duration_by_state_s:
          "POURSUITE": 0.6
        force_switch_states:
          - "RETRAIT (MENACE DIRECTE)"
        log_transitions: true
      params:
        seuil_menace_deg: 15.0
        seuil_lock_tir_deg: 5.0
    """
    path = config_dir / "ia_bt.yaml"
    if not path.exists():
        return {}

    try:
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
            return data
    except Exception as e:
        print(f"[BT] WARNING: impossible de lire ia_bt.yaml: {e}")
        return {}

def main():
    global SEUIL_MENACE_DEG, SEUIL_LOCK_TIR_DEG, ROBOT_AI_ID, ROBOT_ENEMY_ID

    config_dir = Path(__file__).parent.parent / 'config'
    transform_mgr = load_calibration(str(config_dir))

    # Config projecteur
    proj_conf = {}
    if (config_dir / 'projector.yaml').exists():
        with open(config_dir / 'projector.yaml') as f:
            proj_conf = yaml.safe_load(f) or {}

    # Config IA BT (optionnelle)
    ia_conf = _load_bt_config(config_dir)
    bt_conf = ia_conf.get("bt", {}) if isinstance(ia_conf, dict) else {}
    params = ia_conf.get("params", {}) if isinstance(ia_conf, dict) else {}

    # Surcharge paramètres
    if "robot_ai_id" in params:
        ROBOT_AI_ID = int(params["robot_ai_id"])
    if "robot_enemy_id" in params:
        ROBOT_ENEMY_ID = int(params["robot_enemy_id"])
    if "seuil_menace_deg" in params:
        SEUIL_MENACE_DEG = float(params["seuil_menace_deg"])
    if "seuil_lock_tir_deg" in params:
        SEUIL_LOCK_TIR_DEG = float(params["seuil_lock_tir_deg"])

    # Hack SDL pour projecteur
    off_x = proj_conf.get('display', {}).get('monitor_offset_x', 1920)
    os.environ['SDL_VIDEO_WINDOW_POS'] = f"{off_x},0"

    pygame.init()
    screen = pygame.display.set_mode((1024, 768), pygame.NOFRAME)
    font_etat = pygame.font.SysFont("Impact", 50)
    font_debug = pygame.font.SysFont("Consolas", 18)

    camera = RealSenseStream(width=1280, height=720, fps=30)
    camera.start()
    aruco = ArucoDetector()

    world = WorldModel(transform_mgr.arena_width_m, transform_mgr.arena_height_m)
    world.generate_costmap()

    brain = TankBehaviorTree(world, config=bt_conf)

    try:
        while True:
            screen.fill(C_BG)
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    return

            color_frame, _ = camera.get_frames()
            if color_frame is None:
                continue
            detections = aruco.detect(color_frame)

            ai_pose = en_pose = None
            if ROBOT_AI_ID in detections:
                ai_pose = get_robot_pose_in_world(detections[ROBOT_AI_ID], transform_mgr)
            if ROBOT_ENEMY_ID in detections:
                en_pose = get_robot_pose_in_world(detections[ROBOT_ENEMY_ID], transform_mgr)

            mode = brain.update(ai_pose, en_pose)

            if ai_pose and en_pose:
                ai_px = transform_mgr.world_to_projector(ai_pose[0], ai_pose[1])
                en_px = transform_mgr.world_to_projector(en_pose[0], en_pose[1])

                color = C_POURSUITE
                if "RETRAIT" in mode:
                    color = C_RETRAIT
                if "ATTAQUE" in mode:
                    color = C_ATTAQUE

                pygame.draw.line(screen, color, ai_px, en_px, 2)
                pygame.draw.circle(screen, C_CYAN, ai_px, 15, 2)
                pygame.draw.circle(screen, C_RETRAIT, en_px, 15, 2)

            txt_surf = font_etat.render(f"MODE IA : {mode}", True, C_WHITE)
            screen.blit(txt_surf, (screen.get_width()//2 - txt_surf.get_width()//2, 50))

            if ai_pose and en_pose:
                dist = np.hypot(ai_pose[0]-en_pose[0], ai_pose[1]-en_pose[1])
                menace = "OUI" if brain.est_vise_par_ennemi(ai_pose, en_pose) else "NON"
                debug_txt = [
                    f"IA Pose: x={ai_pose[0]:.2f} y={ai_pose[1]:.2f} th={np.degrees(ai_pose[2]):.0f}°",
                    f"EN Pose: x={en_pose[0]:.2f} y={en_pose[1]:.2f} th={np.degrees(en_pose[2]):.0f}°",
                    f"Distance: {dist:.2f}m | Menace: {menace}",
                    f"Seuil menace: {SEUIL_MENACE_DEG:.1f}° | Seuil lock: {SEUIL_LOCK_TIR_DEG:.1f}°",
                ]
                y_y = 650
                for line in debug_txt:
                    s = font_debug.render(line, True, C_WHITE)
                    screen.blit(s, (30, y_y))
                    y_y += 25

            pygame.display.flip()
            pygame.time.Clock().tick(30)

    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()
        pygame.quit()

if __name__ == '__main__':
    main()
