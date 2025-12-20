"""
Moteur de Jeu - Arbitre Central

Ce module orchestre la boucle de jeu et coordonne tous les sous-systèmes.
Il agit comme l'arbitre, gérant :
- Les transitions d'état du jeu
- La gestion des chronomètres
- La validation et la résolution des tirs
- Les conditions de victoire/défaite
- Le cycle de jeu basé sur les ticks (30 FPS)

Le moteur de jeu reçoit :
- L'état du monde (poses des robots, obstacles de la perception)
- Les décisions de l'IA (cible, demande de tir)
- Les entrées humaines (déclencheurs)

Le moteur de jeu produit :
- L'état de jeu mis à jour (scores, temps de recharge, statut)
- Les événements (tirs effectués, touches enregistrées, fin de partie)

Logs : préfixe [GAME] pour tous les événements liés au jeu
"""

class GameEngine:
    """
    Arbitre central du jeu gérant la boucle de jeu et l'application des règles.
    
    Responsabilités :
    - Orchestrer le tick de jeu à 30 FPS
    - Valider et exécuter les tirs (humain + IA)
    - Mettre à jour les chronomètres et temps de recharge
    - Vérifier les conditions de victoire
    - Émettre des événements de jeu pour la visualisation
    
    NE fait PAS :
    - Prendre des décisions IA (ça c'est core/ia/)
    - Contrôler les moteurs (ça c'est core/control/)
    - Dessiner quoi que ce soit (ça c'est visualization/)
    """
    
    
    def __init__(self, config):
        """
        Initialise le moteur de jeu avec la configuration.
        
        Args:
            config: Configuration du jeu depuis config/game.yaml
                   (durée du match, temps de recharge, conditions de victoire)
        """
        self.config = config
        
        # Charge les règles
        self.rules = GameRules.from_config(config['match']) if 'match' in config else GameRules()
        
        # Sous-systèmes auxiliaires
        # Note : Raycast et Hits ont besoin de WorldModel, passé dans tick() ou initialisé plus tard
        # Pour l'instant nous créons des placeholders ou exigeons le monde dans tick
        self.raycast = None 
        self.hit_manager = None
        
        # Chronomètres
        self.cooldowns = CooldownManager(
            self.rules.ai_shot_cooldown, 
            self.rules.human_shot_cooldown
        )
        
        # État
        self.start_time = 0.0
        self.state = GameStatus.READY
        
        print("[GAME] Moteur initialisé")
    
    def _ensure_subsystems(self, world_model):
        """Initialisation paresseuse des sous-systèmes qui ont besoin du modèle du monde."""
        if self.raycast is None:
            from .raycast import Raycast
            from .hits import HitManager
            
            self.raycast = Raycast(world_model.grid)
            self.hit_manager = HitManager(self.raycast)
            print("[GAME] Sous-systèmes liés au WorldModel")

    def tick(self, world_state, ia_request, human_input):
        """
        Exécute un tick de jeu (appelé à 30 FPS).
        
        Args:
            world_state: Instance WorldModel (PAS juste un dict)
            ia_request: Décision IA (cible, demande de tir)
            human_input: Contrôles/déclencheurs humains
            
        Returns:
            dict: État du jeu mis à jour pour la visualisation
        """
        import time
        current_time = time.time()
        
        # Assure que les sous-systèmes sont prêts
        self._ensure_subsystems(world_state)
        
        # 0. Gère les transitions d'état du jeu
        if self.state == GameStatus.READY:
            # Vérifie la condition de démarrage (ex: entrée humaine)
            if human_input.get('start_game', False):
                self.start_time = current_time
                self.state = GameStatus.PLAYING
                self.hit_manager.clear_history()
                print("[GAME] Match DÉMARRÉ")
                
        elif self.state == GameStatus.PLAYING:
            # Vérifie l'expiration du temps
            elapsed = current_time - self.start_time
            if elapsed >= self.rules.match_duration_seconds:
                self.state = GameStatus.FINISHED
                self._check_win_condition()
                print("[GAME] Temps du match ÉCOULÉ")
                
            # 1. Traite les tirs
            self._handle_shooting(world_state, ia_request, human_input, current_time)
            
            # 2. Vérifie la condition de victoire (limite de score)
            game_over, winner = self._check_win_condition()
            if game_over and self.state != GameStatus.FINISHED:
                self.state = GameStatus.FINISHED
                print("[GAME] Match TERMINÉ. Vainqueur : {}".format(winner))

        # 3. Construit le dictionnaire d'état pour la vue
        scores = self.hit_manager.get_score_summary()
        
        state_dict = {
            'status': self.state.value,
            'time_remaining_s': max(0, self.rules.match_duration_seconds - (current_time - self.start_time)) if self.state == GameStatus.PLAYING else 0,
            
            # Scores
            'robot_4_hits_inflicted': scores['robot_4_hits_inflicted'],
            'robot_5_hits_inflicted': scores['robot_5_hits_inflicted'],
            'robot_4_hits_received': scores['robot_4_hits_received'],
            'robot_5_hits_received': scores['robot_5_hits_received'],
            
            # Temps de recharge
            'can_shoot_ai': self.cooldowns.can_shoot_ai(),
            'can_shoot_human': self.cooldowns.can_shoot_human(),
            
            # Info debug transmise
            'ai_has_los': ia_request.get('has_los', False),
            'ai_fire_request': ia_request.get('fire_request', False),
            'ai_state': ia_request.get('state', 'UNKNOWN')
        }
        
        return state_dict
    
    def _handle_shooting(self, world_state, ia_request, human_input, current_time):
        """Traite les demandes de tir de l'IA et de l'Humain."""
        
        # Poses des robots depuis WorldModel
        # On suppose que world_state a des méthodes ou attributs pour les poses
        # Nous avons besoin des poses les plus récentes.
        # Puisque world_state passé ici est l'objet WorldModel, on peut lui demander.
        # Mais attendez, main.py passe `world` qui est WorldModel.
        # On suppose qu'on peut obtenir les poses.
        
        r4_pose = world_state.get_robot_pose(4)
        r5_pose = world_state.get_robot_pose(5)
        
        if r4_pose is None or r5_pose is None:
            return # Impossible de tirer si les robots ne sont pas suivis
            
        # --- TIR IA ---
        if ia_request.get('fire_request', False):
            if self.cooldowns.can_shoot_ai():
                self.cooldowns.register_shot_ai()
                print("[GAME] Tir IA !")
                self.hit_manager.process_shot(4, r4_pose, r5_pose, current_time)
                
        # --- TIR HUMAIN ---
        if human_input.get('fire_request', False):
            if self.cooldowns.can_shoot_human():
                self.cooldowns.register_shot_human()
                print("[GAME] Tir Humain !")
                self.hit_manager.process_shot(5, r5_pose, r4_pose, current_time)

    def _check_win_condition(self):
        """
        Vérifie si le match doit se terminer en fonction des touches.
        
        Returns:
            (game_over: bool, winner: str or None)
        """
        scores = self.hit_manager.get_score_summary()
        ai_hits = scores['robot_4_hits_inflicted']
        human_hits = scores['robot_5_hits_inflicted']
        
        max_hits = self.rules.max_hits_to_win
        
        if ai_hits >= max_hits:
            return True, "AI"
        
        if human_hits >= max_hits:
            return True, "HUMAN"
            
        return False, None

# Imports en bas pour éviter les dépendances circulaires si nécessaire,
# ou en haut si sûr. 
from .rules import GameRules
from .timers import CooldownManager
from .state import GameStatus
