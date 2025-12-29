#!/bin/bash
# Lancement du wizard de calibration standalone
# Usage: ./run_calibration_wizard.sh

# Aller à la racine du module (tank_project)
cd "$(dirname "$0")/.."

echo "=========================================="
echo "  CALIBRATION WIZARD - Tank Arena"
echo "=========================================="
echo ""
echo "Instructions:"
echo "1. Videz l'arène de tout obstacle"
echo "2. Assurez-vous que la caméra voit toute la zone projetée"
echo "3. Préparez un robot (ID 4 ou 5) pour l'étape métrique"
echo ""
echo "Contrôles:"
echo "  ESPACE  - Valider/Continuer"
echo "  ECHAP   - Annuler"
echo ""

python3 -m perception.calibration.standalone_wizard

echo ""
echo "Terminé! Le fichier calibration.json a été créé dans config/"
