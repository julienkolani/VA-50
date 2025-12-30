#!/usr/bin/env python3
import sys
import os
import time
import json
import pygame
import numpy as np
import uuid
from pathlib import Path

# Setup Paths
ROOT_DIR = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT_DIR))

from core.world.unified_transform import load_calibration
from core.utils.state_file import StateFile

# --- CONFIGURATION ---
STATE_FILE = "renderer/game_state.json"

# Couleurs
C_BG = (10, 10, 20)
C_GRID = (30, 30, 50)
C_AI = (0, 150, 255)
C_HUMAN = (255, 50, 0)
C_TEXT = (255, 255, 255)
C_GOLD = (255, 215, 0)
C_LASER = (0, 255, 0)

class GameRenderer:
    """
    Moteur de Rendu du Jeu.
    Peut être utilisé :
    1. En standalone (main) via la lecture du JSON.
    2. Comme librairie importée par le Game Manager (update direct).
    """
    def __init__(self, config_path=None, transform_manager=None, config=None):
        if transform_manager:
            self.tm = transform_manager
        else:
            if not config_path:
                config_path = str(ROOT_DIR / 'config')
            self.tm = load_calibration(config_path)
            
        # Optional: update visual config if provided
        if config:
            # Merge logic or simplistic override
            pass # We will handle this below
        
        # Init Defaults
        proj_w, proj_h = 1024, 768
        proj_off_x = 0
        
        # Default Visuals
        self.vis_cfg = {
            'speed': 800.0,
            'colors': {
                'bg': (10, 10, 20), 'grid': (30, 30, 50),
                'ai': (0, 150, 255), 'human': (255, 50, 0),
                'hitbox': (255, 255, 0), 'text': (255, 255, 255),
                'gold': (255, 215, 0), 'laser': (0, 255, 0)
            },
            'hitbox_r': 0.25
        }

        try:
             import yaml
             # Si config injectée, on l'utilise, sinon on charge du fichier
             if config:
                 cfg = config
             else:
                 cfg_file = Path(config_path) / 'projector.yaml'
                 if cfg_file.exists():
                     with open(cfg_file) as f:
                         cfg = yaml.safe_load(f)
                 else:
                     cfg = {}

             # 1. Display
             proj = cfg.get('projector', {})
             disp = cfg.get('display', {})
             proj_w = proj.get('width', 1024)
             proj_h = proj.get('height', 768)
             proj_off_x = disp.get('monitor_offset_x', 0)
                     
             # 2. Visuals
             vis = cfg.get('visuals', {})
             self.vis_cfg['speed'] = vis.get('projectile_speed_px_s', 800.0)
             
             c = vis.get('colors', {})
             if c:
                self.vis_cfg['colors']['bg'] = tuple(c.get('background', (10,10,20)))
                self.vis_cfg['colors']['grid'] = tuple(c.get('grid', (30,30,50)))
                self.vis_cfg['colors']['ai'] = tuple(c.get('ai', (0,150,255)))
                self.vis_cfg['colors']['human'] = tuple(c.get('human', (255,50,0)))
                self.vis_cfg['colors']['hitbox'] = tuple(c.get('hitbox', (255,255,0)))
                
             hb = vis.get('hitbox', {})
             self.vis_cfg['hitbox_r'] = hb.get('radius_m', 0.25)

        except Exception as e:
            print(f"[RENDER] Config Error: {e}")
        
        os.environ['SDL_VIDEO_WINDOW_POS'] = f"{proj_off_x},0"
        pygame.init()
        self.screen = pygame.display.set_mode((proj_w, proj_h), pygame.NOFRAME)
        self.clock = pygame.time.Clock()
        
        # Assets
        self.font_score = pygame.font.SysFont("Impact", 60)
        self.font_info = pygame.font.SysFont("Consolas", 18)
        self.font_big = pygame.font.SysFont("Impact", 120)
        
        # Internal FX State
        self.floating_texts = []
        self.projectiles = []
        self.processed_events = set()
        
        self.last_ai_score = 0
        self.last_human_score = 0
        self.last_time = time.time()
        
        self.running = True

    def process_events(self):
        """Gère la boucle d'événements Pygame (Quit, etc.). Retourne False si demande d'arrêt."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
        return self.running

    def render(self, state):
        """Dessine une frame en fonction de l'état fourni (Dict)."""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        self.screen.fill(C_BG)
        
        if not state:
            self._draw_overlay("WAITING FOR SIGNAL...")
        else:
            self._process_logic(state, dt)
            self._draw_grid()
            self._draw_entities(state)
            self._draw_fx(dt)
            self._draw_hud(state)
            
        pygame.display.flip()
        self.clock.tick(60)

    def close(self):
        pygame.quit()

    # --- SIMULATION HELPERS (Public API) ---
    def trigger_shot(self, start_pos, end_pos, is_ai):
        """Ajoute un projectile visuel (utile si appelé directement sans passer par JSON event)."""
        self._add_projectile(start_pos, end_pos, is_ai)

    def trigger_hit(self, pos, color=C_GOLD):
        """Ajoute un effet de hit/score."""
        self._add_floating_text("+1", pos[0], pos[1], color)

    # --- INTERNAL RENDERING LOGIC ---
    def _process_logic(self, state, dt):
        # 1. Score auto-animation
        scores = state.get('scores', {})
        s_ai = scores.get('ai', 0)
        s_hu = scores.get('human', 0)
        ents = state.get('entities', {})

        if s_ai > self.last_ai_score:
            pos = ents.get('ai', {}).get('pos', [self.tm.arena_width_m*0.2, 0.5])
            self._add_floating_text("+1", pos[0], pos[1], self.vis_cfg['colors']['ai'])
        
        if s_hu > self.last_human_score:
            pos = ents.get('human', {}).get('pos', [self.tm.arena_width_m*0.8, 0.5])
            self._add_floating_text("+1", pos[0], pos[1], self.vis_cfg['colors']['human'])
            
        self.last_ai_score = s_ai
        self.last_human_score = s_hu
        
        # 2. Events from JSON
        if len(self.processed_events) > 100:
            self.processed_events.clear()
            
        for evt in state.get('events', []):
            eid = evt.get('id')
            if eid and eid in self.processed_events:
                continue 
            if eid: self.processed_events.add(eid)
            
            if evt.get('type') == 'shot':
                p1 = evt.get('shooter_pos')
                p2 = evt.get('target_pos')
                if p1 and p2:
                     self._add_projectile(p1, p2, evt.get('shooter') == 'ai')

    def _draw_grid(self):
        for x in np.arange(0, self.tm.arena_width_m, 0.2):
            p1 = self.tm.world_to_projector(x, 0)
            p2 = self.tm.world_to_projector(x, self.tm.arena_height_m)
            pygame.draw.line(self.screen, self.vis_cfg['colors']['grid'], p1, p2, 1)

    def _draw_robot(self, px_pos, theta, color):
        # Hitbox
        radius_m = self.vis_cfg.get('hitbox_r', 0.25)
        radius_px = int(self.tm.meters_to_pixels(radius_m))
        pygame.draw.circle(self.screen, self.vis_cfg['colors']['hitbox'], (int(px_pos[0]), int(px_pos[1])), radius_px, 1)

        # Corps Robot
        pygame.draw.circle(self.screen, color, (int(px_pos[0]), int(px_pos[1])), 25)
        # Canon
        barrel_len = 50; barrel_width = 10
        c, s = np.cos(theta), np.sin(theta)
        bx, by = px_pos[0] - 10*c, px_pos[1] - 10*s
        tx, ty = px_pos[0] + barrel_len*c, px_pos[1] + barrel_len*s
        px, py = -s * (barrel_width/2), c * (barrel_width/2)
        poly = [(bx+px,by+py), (tx+px,ty+py), (tx-px,ty-py), (bx-px,by-py)]
        pygame.draw.polygon(self.screen, (80, 80, 80), poly)
        pygame.draw.polygon(self.screen, (200, 200, 200), poly, 2)
        pygame.draw.circle(self.screen, (40, 40, 40), (int(px_pos[0]), int(px_pos[1])), 15)

    def _draw_entities(self, state):
        ents = state.get('entities', {})
        if 'ai' in ents:
            pos = ents['ai']['pos']
            px = self.tm.world_to_projector(pos[0], pos[1])
            self._draw_robot(px, pos[2], self.vis_cfg['colors']['ai']) 
        if 'human' in ents:
            pos = ents['human']['pos']
            px = self.tm.world_to_projector(pos[0], pos[1])
            self._draw_robot(px, pos[2], self.vis_cfg['colors']['human'])

    def _draw_hud(self, state):
        scores = state.get('scores', {})
        w = self.screen.get_width()
        
        lbl_ai = self.font_info.render("AI ROBOT", True, self.vis_cfg['colors']['ai'])
        lbl_hu = self.font_info.render("PLAYER", True, self.vis_cfg['colors']['human'])
        self.screen.blit(lbl_ai, (50, 20))
        self.screen.blit(lbl_hu, (w - 50 - lbl_hu.get_width(), 20))

        txt_ai = self.font_score.render(str(scores.get('ai', 0)), True, self.vis_cfg['colors']['ai'])
        self.screen.blit(txt_ai, (50, 50))
        txt_hu = self.font_score.render(str(scores.get('human', 0)), True, self.vis_cfg['colors']['human'])
        self.screen.blit(txt_hu, (w - 50 - txt_hu.get_width(), 50))
        
        status = state.get('status', 'WAITING')
        if status != 'RUNNING':
            self._draw_overlay(status)

    def _draw_overlay(self, text):
        s = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)
        s.fill((0, 0, 0, 150))
        self.screen.blit(s, (0,0))
        txt = self.font_big.render(text, True, C_GOLD) # Keep Gold for overlay
        cx, cy = self.screen.get_width()//2, self.screen.get_height()//2
        self.screen.blit(txt, (cx - txt.get_width()//2, cy - txt.get_height()//2))

    def _add_floating_text(self, text, wx, wy, color):
        px = self.tm.world_to_projector(wx, wy)
        self.floating_texts.append({
            'text': text, 'x': px[0], 'y': px[1] - 50, 'life': 40, 'color': color
        })
        
    def _add_projectile(self, p1_world, p2_world, is_ai):
        start = self.tm.world_to_projector(p1_world[0], p1_world[1])
        end = self.tm.world_to_projector(p2_world[0], p2_world[1])
        dx = end[0] - start[0]; dy = end[1] - start[1]
        dist = np.sqrt(dx**2 + dy**2)
        if dist < 1: return
        
        SPEED = self.vis_cfg.get('speed', 800.0)
        vx = (dx/dist)*SPEED; vy = (dy/dist)*SPEED
        color = self.vis_cfg['colors']['ai'] if is_ai else self.vis_cfg['colors']['human']
        self.projectiles.append({
             'pos': [start[0], start[1]], 'vel': [vx, vy], 'color': color, 
             'life': dist/SPEED
        })

    def _draw_fx(self, dt):
        for p in self.projectiles[:]:
            p['life'] -= dt
            p['pos'][0] += p['vel'][0] * dt
            p['pos'][1] += p['vel'][1] * dt
            if p['life'] <= 0:
                self.projectiles.remove(p); continue
            px, py = p['pos']
            pygame.draw.circle(self.screen, p['color'], (int(px), int(py)), 8)
            pygame.draw.circle(self.screen, (255, 255, 255), (int(px), int(py)), 3)
        
        for ft in self.floating_texts[:]:
            ft['life'] -= 1; ft['y'] -= 1
            if ft['life'] <= 0:
                self.floating_texts.remove(ft); continue
            rend = self.font_score.render(ft['text'], True, ft['color'])
            self.screen.blit(rend, (int(ft['x']), int(ft['y'])))


# --- STANDALONE / MANUAL SIMULATION ---

class ManualSimulator:
    def __init__(self, tm):
        self.tm = tm
        self.ai_pos = [tm.arena_width_m * 0.2, tm.arena_height_m * 0.5, 0.0]  
        self.hu_pos = [tm.arena_width_m * 0.8, tm.arena_height_m * 0.5, 3.14]
    
    def update(self, state, action):
        if not state:
            state = {
                "timestamp": time.time(), "status": "WAITING", 
                "scores": { "ai": 0, "human": 0 },
                "entities": {
                    "ai": { "pos": self.ai_pos },
                    "human": { "pos": self.hu_pos }
                }, "events": []
            }
        
        state['timestamp'] = time.time()
        state['events'] = [] 
        evt_id = str(uuid.uuid4())
        
        if action == 'start': state['status'] = 'RUNNING'; state['scores'] = { "ai": 0, "human": 0 }
        elif action == 'end': state['status'] = 'GAME OVER'
        elif state['status'] == 'RUNNING':
            if action == 'shoot_ai':
                state['events'].append({ 'id': evt_id, 'type': 'shot', 'shooter': 'ai', 'shooter_pos': self.ai_pos, 'target_pos': self.hu_pos })
            elif action == 'shoot_human':
                state['events'].append({ 'id': evt_id, 'type': 'shot', 'shooter': 'human', 'shooter_pos': self.hu_pos, 'target_pos': self.ai_pos })
            elif action == 'hit':
                state['scores']['ai'] += 1 
                state['events'].append({ 'id': evt_id, 'type': 'hit', 'target': 'human' })

        sf = StateFile(STATE_FILE)
        sf.write(state)
        return state

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', action='store_true', help="Mode Simulation Manuelle")
    args = parser.parse_args()
    
    renderer = GameRenderer()
    
    # Standalone Loop
    sim = ManualSimulator(renderer.tm) if args.sim else None
    sim_state = None
    
    # Init SIM
    if sim:
        print("[Manual Sim] Controls: SPACE=Start, ENTER=End, A/Z=Shoot, E=Hit")
        sim_state = sim.update(None, None)
    
    sf = StateFile(STATE_FILE)
    
    while renderer.running:
        # Input for Sim
        if not renderer.process_events():
            break
            
        keys = pygame.key.get_pressed()
        # Note: process_events consumant les events, on devrait plutôt hooker GameRenderer. 
        # Mais pour ce test rapide, on va patcher process_events ou juste lire l'état global si on veut...
        # Mieux : on réimplémente une lecture d'event basique ici ou on modifie GameRenderer pour exposer les events.
        # Pour faire simple et propre dans le style "Library", GameRenderer devrait juste rendre.
        # Le Input Handling spécifique à la Sim est un hack de debug.
        
        # Hack direct sur pygame pour la simu (puisque renderer.process_events vide la queue)
        # On va plutôt faire un petit switch dans renderer pour récupérer les inputs si besoin?
        # Non, on va laisser GameRenderer gérer le Quit, et on va juste chequer les touches ici si on peut.
        # pygame.key.get_pressed() marche toujours.
        
        if sim:
            action = None
            if keys[pygame.K_SPACE]: action = 'start'
            if keys[pygame.K_RETURN]: action = 'end'
            if keys[pygame.K_a]: action = 'shoot_ai'
            if keys[pygame.K_z]: action = 'shoot_human'
            if keys[pygame.K_e]: action = 'hit'
            
            # Debounce très basic (sinon ça mitraille à 60fps)
            if action and (time.time() - getattr(main, 'last_act', 0) > 0.2):
                sim_state = sim.update(sim_state, action)
                main.last_act = time.time()

        # Read State
        state = sf.read()
        renderer.update(state)
        
    renderer.close()

if __name__ == '__main__':
    main()
