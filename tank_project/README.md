# Tank Project

Moteur de jeu principal pour Tank Arena avec IA comportementale, vision par ordinateur, et contrôle robot.

## Structure

```
tank_project/
├── config/              # Configuration YAML
├── core/
│   ├── control/         # Cinématique, trajectoire, pont ROS
│   ├── game/            # Moteur de Jeu, Raycast, Gestion des touches
│   ├── ia/              # Arbre de comportement, A*, Décisions
│   └── world/           # Modèle du Monde, Grille d'occupation
├── perception/
│   ├── calibration/     # Calibration arène/projecteur
│   └── camera/          # ArUco, Kalman, RealSense
├── visualization/       # Rendu Pygame, HUD
└── scripts/
    └── run_game.py      # Point d'entrée
```

## Lancement

```bash
cd tank_project
python3 scripts/run_game.py
```

## Configuration

| Fichier       | Description                  |
| ------------- | ---------------------------- |
| `arena.yaml`  | Dimensions arène, projecteur |
| `camera.yaml` | RealSense, ArUco, Kalman     |
| `game.yaml`   | Règles du jeu, cooldowns     |
| `ia.yaml`     | Comportement IA              |
| `robot.yaml`  | Cinématique, port 8765       |

## Connexion au Bridge

Le `ROSBridgeClient` se connecte au Safety Bridge avec retry automatique :

```python
client = ROSBridgeClient(host='localhost', port=8765)
client.connect(max_retries=0, retry_interval=8.0)  # Retry infini
```

## Contrôles

| Touche    | Action                     |
| --------- | -------------------------- |
| `Espace`  | Démarrer match             |
| `Flèches` | Contrôler robot humain     |
| `F`       | Tirer                      |
| `D`       | Toggle debug paths / inflation |
| `ESC`     | Quitter                    |

## Scripts Utiles

```bash
# Calibration (obligatoire 1ère fois)
python3 scripts/run_calibration.py

# Validation visuelle homographie
python3 scripts/show_grid.py

# Lancer le jeu
python3 scripts/run_game.py
```
