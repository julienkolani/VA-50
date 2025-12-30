# Tank Arena - Wiki du Projet

Ce projet orchestre un jeu de "Laser Tag" en réalité mixte entre un Robot Autonome (IA) et un Robot Joueur Humain, utilisant un projecteur pour le sol de l'arène et des caméras au plafond pour le suivi de position.

---

## Architecture Générale

Le projet a évolué d'un script monolithique (`run_game.py`) vers une architecture modulaire pilotée par événements.

### Composants Centraux

*   **`main.py`** : Le point d'entrée. Initialise le système et lance le Manager.
    *   *Usage* : `python3 main.py` (Réel) ou `python3 main.py --mock` (Simulation).
*   **`core/game/game_manager.py` (Le Cerveau)** : 
    *   **Rôle** : Orchestrateur Central. Il possède la boucle principale.
    *   **Responsabilités** :
        *   Fusionne les données de Perception (Caméra/ArUco).
        *   Met à jour le Modèle du Monde (Physique, Occupation).
        *   Interroge la Stratégie IA.
        *   Envoie les commandes aux Robots (Pont ROS).
        *   Met à jour la Visualisation (Renderer).
*   **`core/ia/strategy.py` (Le Stratège)** :
    *   **Rôle** : Décide *Quoi faire* (Attaquer, Fuir, Explorer).
    *   **Sortie** : État de haut niveau et vecteurs de vitesse bas niveau (`v`, `w`).
*   **`core/control/trajectory_follower.py` (Le Pilote)** :
    *   **Rôle** : Convertit les points de passage (waypoints) en Commandes Moteur via **Pure Pursuit**.
    *   **Logique Clé** : Inclut une "Hystérésis d'Alignement" pour empêcher la **Singularité 180°** (voir Dépannage).
*   **`renderer/game_renderer.py` (L'Artiste)** :
    *   **Rôle** : Dessine l'état du jeu au sol avec Pygame.
    *   **Note** : Découplé de la logique. Lit l'état et dessine simplement.

---

## Guide de Configuration

Toutes les "Valeurs" sont externalisées dans `config/*.yaml`.

### 1. `game.yaml` (Règles & Physique)
*   `duration` : 180s (3 min).
*   `shooting.hit_radius` : **0.25m**. Valeur critique ! C'est la taille effective du robot pour la détection de touche.
*   `mock` : Positions de départ pour le mode simulation.

### 2. `robot.yaml` (Contrôle & Limites)
*   **Pure Pursuit** :
    *   `lookahead_distance` : 0.15m. Distance à "viser". Attention : Trop grand = coupe les virages. Trop petit = oscillations.
    *   `k_v` : 0.6. Gain de vitesse.
*   **Alignement (L'Anti-Spin)** :
    *   `threshold_angle` : 45°. Si la cible est derrière (>45°), STOP mouvement et rotation sur place.
    *   `hysteresis_angle` : 10°. Reprend le mouvement une fois aligné.
*   **Limites** :
    *   `max_linear_vel` : 0.22 m/s. Limite matérielle sûre.
    *   `max_angular_vel` : 1.2 rad/s.

### 3. `projector.yaml` (Visuels)
*   `visuals.colors` : Valeurs RGB personnalisées.
*   `projectile_speed_px_s` : 800. Vitesse des tirs laser.

---

## Calibration

Le **UnifiedTransform** (`core/world/unified_transform.py`) gère la conversion entre :
1.  **Pixels Caméra** (Entrée RealSense)
2.  **Mètres Monde** (Physique/Logique Robot)
3.  **Pixels Projecteur** (Sortie Pygame)

Il utilise une **Matrice d'Homographie** chargée depuis `config/calibration.npz`.
*   **Astuce** : Si les projections semblent "dérivées", relancez `run_calibration_wizard.sh`.

---

## Dépannage & Historique (Le "Pourquoi")

### 1. Le "Spin Infini" (Singularité 180°)
*   **Symptôme** : Le robot tournait parfois sans fin quand la cible était juste derrière lui.
*   **Cause** : Le calcul Pure Pursuit est instable à 180°. Il essaie de tourner en avançant, créant un cercle qui n'atteint jamais le point.
*   **Correctif** : Implémentation d'une **Machine à État Hybride** dans `TrajectoryFollower`.
    *   SI angle > 45° -> **Stop & Rotation Seule**.
    *   Cela force le robot à faire face à la cible avant de tenter d'avancer.

### 2. Confusion d'ID (Se Tirer Dessus)
*   **Symptôme** : L'utilisateur appuie sur "Tirer", mais c'est le robot IA qui tire.
*   **Cause** : Les robots physiques avaient les marqueurs ArUco inversés par rapport aux hypothèses du code.
    *   Le code pensait ID 4 = IA. La réalité avait ID 4 sur le robot Utilisateur.
*   **Correctif** : Inversion forcée dans `GameManager` (ID 4 = HUMAIN, ID 5 = IA). Vérifiez toujours les IDs avec `scripts/find_aruco_front.py`.

### 3. Mouvements Fantômes (Filtre de Kalman)
*   **Symptôme** : Caméra bloquée -> Le robot se "téléporte" en 0,0.
*   **Correctif** : Implémentation de **Filtres de Kalman** (`core/control/kalman.py`).
    *   Si la perception est perdue, la prédiction de vitesse maintient le mouvement fluide pendant ~1s.
    *   Ajout du compteur `ai_lost_frames` pour avertir si le suivi est perdu >1s.

---

## Démarrage Rapide

1.  **Vérification Matériel** : Connectez RealSense & Projecteur.
2.  **Lancer le Pont ROS** (dans un terminal séparé) pour parler aux robots.
3.  **Lancer** : `python3 main.py`
4.  **Contrôles** :
    *   **ESPACE** : Démarrer/Pause IA.
    *   **F**, **D**, **S**, **Q** : Tir Joueur (Directionnel).
