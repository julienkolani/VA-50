# Guide Rapide - Lancement du Projet

## Methodes de Lancement

### 1. Depuis le repertoire du projet

```bash
cd /home/julien/ros2_ws/src/VA-51/tank_project

# Afficher l'aide
python3 main.py --help

# Lancer le jeu
python3 main.py
python3 main.py game

# Lancer la calibration
python3 main.py calibration

# Exporter donnees debug
python3 main.py export
```

### 2. En tant que module Python

```bash
cd /home/julien/ros2_ws/src/VA-51

# Afficher l'aide
python3 -m tank_project --help

# Lancer le jeu
python3 -m tank_project
python3 -m tank_project game

# Lancer la calibration
python3 -m tank_project calibration

# Exporter donnees debug
python3 -m tank_project export
```

### 3. Via les scripts directs

```bash
cd /home/julien/ros2_ws/src/VA-51/tank_project

# Calibration
python3 scripts/run_calibration.py

# Jeu
python3 scripts/run_game.py

# Export debug
python3 scripts/export_debug_data.py --output-dir logs/debug_custom
```

## Prerequisites

### Installation des dependances

```bash
cd /home/julien/ros2_ws/src/VA-51/tank_project
pip3 install -r requirements.txt
```

Dependances principales:
- `numpy` - Calculs numeriques
- `opencv-python` - Vision par ordinateur
- `pyrealsense2` - Interface camera RealSense
- `pygame` - Rendu graphique
- `scipy` - Traitement signal
- `pyyaml` - Configuration

## Workflow Complet

### Premiere Utilisation

```bash
# 1. Installer dependances
pip3 install -r requirements.txt

# 2. Lancer calibration (obligatoire la premiere fois)
python3 scripts/run_calibration.py

# Suivre les instructions a l'ecran:
# - Appuyer sur ESPACE pour valider chaque etape
# - Detecter coins projetes (ArUco 0-3)
# - Mesurer marqueur physique (ArUco 4 ou 5)
# - Cartographier obstacles

# 3. Verifier calibration sauvegardee
cat config/arena.yaml

# 4. Valider visuellement l'homographie (RECOMMANDE)
python3 scripts/show_grid.py
# - Poser un marqueur dans un coin -> la cible cyan doit etre au bon endroit
# - Appuyer sur D pour voir l'inflation (zones oranges)
# - ESC pour quitter

# 5. Lancer le jeu
python3 scripts/run_game.py
```

### Utilisation Normale

```bash
# Lancer directement le jeu (calibration deja faite)
python3 main.py
```

## Debug et Diagnostic

### Export donnees debug

```bash
# Exporter snapshot complet
python3 main.py export

# Donnees exportees dans:
# logs/debug_YYYYMMDD_HHMMSS/
```

### Verifier imports

```bash
# Tester que tous les modules s'importent correctement
python3 -c "
from core.game import game_engine
from core.ia import strategy
from perception.camera import aruco_detector
from visualization import pygame_renderer
print('Tous les imports OK')
"
```

### Logs

Les logs sont stockes dans `logs/`:
- `runtime.log` - Logs d'execution jeu
- `calibration.log` - Logs calibration
- `debug.log` - Logs debug general

Filtrer logs par module:
```bash
# Logs IA uniquement
python3 main.py 2>&1 | grep "\[AI\]"

# Logs vision uniquement
python3 main.py 2>&1 | grep "\[VISION\]"

# Logs calibration
python3 main.py calibration 2>&1 | tee logs/calibration.log
```

## Configuration

Modifier les parametres dans `config/*.yaml`:

- `arena.yaml` - Dimensions arene, transformations, affichage
- `camera.yaml` - Parametres RealSense, ArUco
- `game.yaml` - Regles jeu (duree, cooldowns)
- `ia.yaml` - Comportement IA (distances, seuils)
- `robot.yaml` - Specs Turtlebot (vitesses, dimensions)

Exemple:
```bash
# Editer duree match (defaut: 180s)
nano config/game.yaml
```

## Depannage

### Erreur "No module named 'cv2'"
```bash
pip3 install opencv-python
```

### Erreur "No module named 'pyrealsense2'"
```bash
# Installer SDK RealSense
sudo apt-get install librealsense2-dev
pip3 install pyrealsense2
```

### Pygame ne demarre pas
```bash
pip3 install --upgrade pygame
```

### ROS bridge connection failed
Verifier que le pont ROS est actif sur les robots:
```bash
# Sur le robot
ros2 run ros_bridge server
```

## Plus d'Informations

Voir le [README.md](README.md) complet pour:
- Architecture detaillee
- Description modules
- Mecaniques de jeu
- Details techniques
