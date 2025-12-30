# Tank Arena - Architecture Unifiée

**Tank Arena** est un jeu en Réalité Mixte où des robots autonomes affrontent des joueurs humains dans une arène projetée au sol.

> **Documentation**
> Pour les détails d'Architecture, Configuration et Dépannage, voir le [**wiki.md**](./wiki.md).

## Démarrage

### Prérequis
*   Python 3.8+
*   Pygame, OpenCV, NumPy
*   Serveur ROS Bridge (lancé pour le contrôle robot)

### Lancer le Jeu

**1. Mode Matériel Réel** (Projecteur + Caméra + Robots)
```bash
python3 main.py
```

**2. Mode Simulation** (Sans matériel)
```bash
python3 main.py --mock
```

## Structure du Projet

*   `main.py` : Point d'entrée.
*   `core/` : Logique Jeu (Manager), Contrôle (ROS), IA (Stratégie).
*   `renderer/` : Visualisation Pygame.
*   `perception/` : Vision par Ordinateur (RealSense + ArUco).
*   `config/` : Fichiers de configuration YAML.

## Notes Importantes
*   **IDs Robots** : 
    *   **ID 4** : Joueur HUMAIN (Rouge)
    *   **ID 5** : Robot IA (Bleu)
*   **Calibration** : Si la projection est décalée, lancez `scripts/run_calibration_wizard.sh`.
