#!/usr/bin/env python3
"""
Calibration Wizard Runner

Interactive calibration process to set up the arena:
1. Safe zone definition
2. Geometric calibration (projected corners)
3. Metric calibration (physical marker)
4. Obstacle mapping

Saves results to config/arena.yaml

Usage:
    python3 run_calibration.py
    
Prerequisites:
    - RealSense camera connected
    - Projector showing Pygame window with projected ArUco corners
"""

import sys
import yaml
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.camera.realsense_stream import RealSenseStream
from perception.calibration.calibration_wizard import CalibrationWizard


def main():
    print("[CALIB_RUNNER] ========== Calibration Wizard ==========")
    
    # Load configuration
    print("[CALIB_RUNNER] Loading configuration...")
    config_dir = Path(__file__).parent.parent / 'config'
    
    with open(config_dir / 'camera.yaml') as f:
        camera_config = yaml.safe_load(f)
    
    projector_config_path = config_dir / 'projector.yaml'
    if not projector_config_path.exists():
        print("[CALIB_RUNNER] ERREUR : 'config/projector.yaml' introuvable !")
        print("[CALIB_RUNNER] Veuillez d'abord lancer : python3 scripts/detect_projector_resolution.py")
        sys.exit(1)
        
    with open(projector_config_path) as f:
        projector_config = yaml.safe_load(f)
    
    # Initialize camera with config
    print("[CALIB_RUNNER] Initializing camera...")
    camera = RealSenseStream(
        width=camera_config['realsense']['width'],
        height=camera_config['realsense']['height'],
        fps=camera_config['realsense']['fps']
    )
    camera.start()
    
    # Run wizard with projector config
    proj = projector_config['projector']
    disp = projector_config['display']
    calib = projector_config['calibration']
    
    wizard = CalibrationWizard(
        camera, 
        projector_width=proj['width'],
        projector_height=proj['height'],
        margin_px=proj['margin_px'],
        monitor_offset_x=disp['monitor_offset_x'],
        monitor_offset_y=disp['monitor_offset_y'],
        borderless=disp['borderless'],
        hide_cursor=disp['hide_cursor'],
        marker_size_m=calib['marker_size_m']
    )
    
    try:
        print("[CALIB_RUNNER] Running calibration wizard...")
        results = wizard.run()
        print("[CALIB_RUNNER] Calibration wizard completed!")
        
        # Save to config
        config_path = Path(__file__).parent.parent / 'config' / 'arena.yaml'
        
        print("[CALIB_RUNNER] Saving calibration to {}".format(config_path))
        
        # Convert numpy types to Python native types for clean YAML
        import numpy as np
        
        def numpy_to_python(obj):
            """Recursively convert numpy types to Python native types."""
            if isinstance(obj, dict):
                return {k: numpy_to_python(v) for k, v in obj.items()}
            elif isinstance(obj, list):
                return [numpy_to_python(item) for item in obj]
            elif isinstance(obj, np.ndarray):
                return numpy_to_python(obj.tolist())
            elif isinstance(obj, (np.floating, np.float64, np.float32)):
                return float(obj)
            elif isinstance(obj, (np.integer, np.int64, np.int32)):
                return int(obj)
            else:
                return obj
        
        clean_results = numpy_to_python(results)
        
        with open(config_path, 'w') as f:
            yaml.dump(clean_results, f, default_flow_style=False)
        
        print("[CALIB_RUNNER] Calibration saved successfully!")
        print("[CALIB_RUNNER] You can now run the game with: python3 run_game.py")
        
    except Exception as e:
        print("[CALIB_RUNNER] ERROR: {}".format(e))
        
    finally:
        camera.stop()
        print("[CALIB_RUNNER] Done")


if __name__ == '__main__':
    main()
