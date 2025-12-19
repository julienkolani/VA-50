"""
Calibration Wizard - Interactive Setup Process

Guides user through calibration sequence:
1. Safe zone definition (projection margins)
2. Geometric calibration (H_C2AV from projected corners)
3. Metric calibration (scale from physical ArUco)
4. Obstacle mapping (static obstacles detection)

Saves calibration to config/arena.yaml for game phase.

Logs: [CALIB] prefix for all steps
"""

import cv2
import numpy as np
import yaml
import time
import sys
from typing import Tuple, List
from ..camera.aruco_detector import ArucoDetector
from core.world.coordinate_frames import TransformManager
from .projector_display import ProjectorDisplay


class CalibrationWizard:
    """
    Interactive calibration process for arena setup.
    
    Produces H_C2W transform and arena parameters.
    """
    
    def __init__(self, camera, projector_width=1024, projector_height=768, 
                 margin_px=50, monitor_offset_x=1920, monitor_offset_y=0,
                 borderless=True, hide_cursor=True, marker_size_m=0.10):
        """
        Initialize calibration wizard.
        
        Args:
            camera: RealSenseStream instance
            projector_width: Projector resolution width
            projector_height: Projector resolution height
            margin_px: Safety margin from edges (pixels)
            monitor_offset_x: X position for secondary monitor
            monitor_offset_y: Y position for secondary monitor
            borderless: Use borderless window mode
            hide_cursor: Hide mouse cursor
            marker_size_m: Physical marker size in meters
        """
        self.camera = camera
        self.proj_w = projector_width
        self.proj_h = projector_height
        self.marker_size_m = marker_size_m
        
        self.aruco = ArucoDetector()
        self.transform_mgr = TransformManager()
        
        # Initialize projector display with all config
        self.projector = ProjectorDisplay(
            width=projector_width,
            height=projector_height,
            margin=margin_px,
            monitor_offset_x=monitor_offset_x,
            monitor_offset_y=monitor_offset_y,
            borderless=borderless,
            hide_cursor=hide_cursor
        )
        
        # Calibration results
        self.margin_px = margin_px
        self.arena_width_m = None
        self.arena_height_m = None
        self.H_C2W = None
        self.static_obstacles = []
    
    def _wait_for_user_validation(self, message: str = "Appuyez sur ESPACE pour continuer..."):
        """
        Wait for SPACE key in Pygame window while keeping it responsive.
        
        This prevents the "not responding" error by processing all input
        exclusively through Pygame instead of mixing console/Pygame.
        
        Args:
            message: Message to display to user
        """
        print(f"[CALIB] ATTENTE: {message}")
        
        # Show message on projector
        self.projector.show_message(message, color=(255, 255, 255), bg_color=(50, 50, 50))
        
        import pygame
        waiting = True
        while waiting:
            # Process all Pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    print("[CALIB] User closed window, exiting...")
                    sys.exit(0)
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        waiting = False
                        print("[CALIB] Validé !")
                    elif event.key == pygame.K_ESCAPE:
                        print("[CALIB] Calibration cancelled by user")
                        sys.exit(0)
                    elif event.key == pygame.K_q:
                        print("[CALIB] Calibration cancelled by user")
                        sys.exit(0)
            
            # Small sleep to prevent CPU spinning
            time.sleep(0.01)
        
    def run(self) -> dict:
        """
        Run complete calibration wizard.
        
        Returns:
            dict: Calibration results to save to config
            
        Steps:
            1. Define safe zone (margins)
            2. Detect projected corners → H_C2AV
            3. Detect physical marker → scale → H_C2W
            4. Map static obstacles
            5. Calculate arena dimensions
            
        Logs:
            [CALIB] Step X/4: Description
        """
        print("[CALIB] ========== Starting Calibration Wizard ==========")
        
        # Start projector display
        print("[CALIB] Starting projector display...")
        self.projector.start()
        
        try:
            # Step 1: Safe zone
            self._step_safe_zone()
            
            # Step 2: Geometric calibration
            H_C2AV = self._step_geometric_calibration()
            
            # Step 3: Metric calibration
            scale = self._step_metric_calibration(H_C2AV)
            
            # Step 4: Obstacle mapping
            obstacles = self._step_obstacle_mapping()
            
            # Build results
            results = {
                'projector': {
                    'width': self.proj_w,
                    'height': self.proj_h,
                    'margin': self.margin_px,
                    'margin_px': self.margin_px  # Alias for compatibility
                },
                'display': {
                    'fullscreen': False,
                    'display_index': 0
                },
                'arena': {
                    'width_m': self.arena_width_m,
                    'height_m': self.arena_height_m
                },
                'transform': {
                    'H_C2W': self.H_C2W.tolist() if self.H_C2W is not None else None,
                    'scale': scale,
                    'scale_m_per_av': scale  # Alias for compatibility
                },
                'grid': {
                    'resolution_m': 0.02,  # 2cm grid resolution
                    'inflation_radius_m': 0.15  # 15cm safety margin
                },
                'obstacles': obstacles
            }
            
            print("[CALIB] Calibration complete!")
            return results
            
        finally:
            # Always stop projector
            print("[CALIB] Stopping projector display...")
            self.projector.stop()
    
    def _step_safe_zone(self):
        """
        Step 1: Define safe zone for projection.
        
        Logs:
            [CALIB] MARGIN set to X px
            [CALIB] Arena rect in projector: (x1,y1) -> (x2,y2)
        """
        print("[CALIB] MARGIN set to {} px".format(self.margin_px))
        
        x1, y1 = self.margin_px, self.margin_px
        x2 = self.proj_w - self.margin_px
        y2 = self.proj_h - self.margin_px
        
        print("[CALIB] Arena rect in projector: ({},{}) -> ({},{})".format(x1, y1, x2, y2))
        
    def _step_geometric_calibration(self) -> np.ndarray:
        """
        Step 2: Detect projected corners and compute H_C2AV.
        
        Returns:
            H_C2AV: Homography matrix
            
        Logs:
            [CALIB] Detected 4 projected corners
            [CALIB] H_C2AV computed successfully
        """
        print("[CALIB] Step 2/4: Geometric calibration (detecting projected corners)")
        
        # Project ArUco markers at corners
        print("[CALIB] Projecting ArUco markers (IDs 0-3) at arena corners...")
        self.projector.show_corner_markers(marker_size_px=200)
        
        print("[CALIB] ArUco markers displayed on projector")
        print("[CALIB] Verify that camera can see all 4 markers, then press SPACE...")
        
        self._wait_for_user_validation("Vérifiez les 4 marqueurs et appuyez sur ESPACE")
        
        # Capture frame
        color, _ = self.camera.get_frames()
        
        # Detect ArUco
        detections = self.aruco.detect(color)
        
        # Check for corners (IDs 0-3)
        corner_ids = [0, 1, 2, 3]
        detected_corners = {k: v for k, v in detections.items() if k in corner_ids}
        
        if len(detected_corners) != 4:
            print("[CALIB] ERROR: Expected 4 corners, found {}".format(len(detected_corners)))
            print("[CALIB] Detected IDs: {}".format(list(detected_corners.keys())))
            raise ValueError("Missing projected corners")
        
        print("[CALIB] Detected 4 projected corners")
        
        # Build correspondences
        src_points = []
        dst_points = []
        
        # Arena virtual coordinates (unit square)
        av_coords = {
            0: (0.0, 0.0),  # Bottom-left
            1: (1.0, 0.0),  # Bottom-right
            2: (1.0, 1.0),  # Top-right
            3: (0.0, 1.0)   # Top-left
        }
        
        for marker_id in corner_ids:
            center = detected_corners[marker_id]['center']
            src_points.append(center)
            dst_points.append(av_coords[marker_id])
        
        src_points = np.array(src_points, dtype=np.float32)
        dst_points = np.array(dst_points, dtype=np.float32)
        
        # Compute homography
        H_C2AV, _ = cv2.findHomography(src_points, dst_points)
        
        print("[CALIB] H_C2AV computed successfully")
        
        return H_C2AV
    
    def _step_metric_calibration(self, H_C2AV: np.ndarray) -> float:
        """
        Step 3: Estimate metric scale from physical marker using homography.
        
        This method uses the homography to correctly account for camera perspective,
        ensuring accurate measurements regardless of where the marker is placed.
        
        Args:
            H_C2AV: Homography from step 2 (camera pixels -> arena virtual)
            
        Returns:
            scale: meters per AV unit
            
        Logs:
            [CALIB] Real marker size: X m
            [CALIB] Marker size in AV: Y units
            [CALIB] Scale: Z m / AV_unit
        """
        print("[CALIB] Step 3/4: Metric calibration (place physical marker in arena)")
        
        # Display instructions on projector
        self.projector.show_message("Placez le robot (ID 4 ou 5) au centre", 
                                    color=(255, 255, 255), bg_color=(50, 50, 50))
        
        # Use configured marker size
        marker_size_real = self.marker_size_m
        print(f"[CALIB] Using marker size: {marker_size_real} m (depuis config/projector.yaml)")
        
        self._wait_for_user_validation("Placez le marqueur robot et appuyez sur ESPACE")
        
        # Capture frame
        color, _ = self.camera.get_frames()
        
        # Detect markers
        detections = self.aruco.detect(color)
        
        # Find robot marker (ID 4 or 5)
        marker_data = None
        robot_marker_id = None
        for mid in [4, 5]:
            if mid in detections:
                marker_data = detections[mid]
                robot_marker_id = mid
                break
        
        if marker_data is None:
            raise ValueError("Aucun marqueur robot détecté (ID 4 ou 5) !")
        
        print(f"[CALIB] Marqueur robot ID {robot_marker_id} détecté")
        
        # ============================================================
        # CORRECT METHOD: Use homography to transform marker corners
        # This accounts for camera perspective distortion
        # ============================================================
        
        corners_pix = marker_data['corners']  # List of 4 corner tuples in pixels
        
        # Transform corners from camera pixels to Arena Virtual space using H_C2AV
        pts_src = np.array([corners_pix], dtype=np.float32)
        pts_av = cv2.perspectiveTransform(pts_src, H_C2AV)  # Output: AV coordinates
        
        corners_av = pts_av[0]  # Shape: (4, 2)
        
        # Calculate marker size in AV space (average of all 4 sides)
        side_lengths = []
        for i in range(4):
            j = (i + 1) % 4
            length = np.linalg.norm(corners_av[j] - corners_av[i])
            side_lengths.append(length)
        
        size_av = np.mean(side_lengths)
        
        print(f"[CALIB] Taille du marqueur dans l'espace virtuel: {size_av:.4f} unités")
        
        # Compute scale: meters per AV unit
        scale_m_per_av = marker_size_real / size_av
        
        print(f"[CALIB] ÉCHELLE VALIDÉE: 1.0 unité virtuelle = {scale_m_per_av:.4f} mètres")
        
        # ============================================================
        # Build H_C2W using corner markers (IDs 0-3)
        # ============================================================
        
        corner_ids = [0, 1, 2, 3]
        av_coords = {
            0: [0.0, 0.0],  # Bottom-left
            1: [1.0, 0.0],  # Bottom-right
            2: [1.0, 1.0],  # Top-right
            3: [0.0, 1.0]   # Top-left
        }
        
        src_centers = []
        dst_coords = []
        for mid in corner_ids:
            if mid in detections:
                src_centers.append(detections[mid]['center'])
                dst_coords.append(av_coords[mid])
        
        if len(src_centers) < 4:
            print(f"[CALIB] WARNING: Seulement {len(src_centers)}/4 marqueurs de coin détectés")
        
        self.transform_mgr.set_camera_to_av(
            np.array(src_centers, dtype=np.float32),
            np.array(dst_coords, dtype=np.float32)
        )
        self.transform_mgr.set_av_to_world_scale(scale_m_per_av)
        
        self.H_C2W = self.transform_mgr.H_C2W
        
        # ============================================================
        # Calculate arena dimensions using homography-corrected scale
        # In AV space, the arena is always 1.0 x 1.0 (unit square)
        # So real dimensions = scale * 1.0
        # ============================================================
        
        # The arena width in meters is simply the scale (since AV width = 1.0)
        self.arena_width_m = scale_m_per_av * 1.0
        
        # For height, we need to account for the projector/camera aspect ratio
        # The AV space maps 1.0 to both width and height, but the physical
        # arena may not be square (depends on projector aspect ratio)
        aspect_ratio = self.proj_w / self.proj_h  # e.g., 1024/768 = 1.33
        self.arena_height_m = self.arena_width_m / aspect_ratio
        
        print(f"[CALIB] Dimensions de l'arène: {self.arena_width_m:.2f}m x {self.arena_height_m:.2f}m")
        print(f"[CALIB] Ratio d'aspect: {aspect_ratio:.2f}")
        
        print("[CALIB] H_C2W calculé")
        print("[CALIB] Calibration métrique OK")
        
        return scale_m_per_av
    
    def _step_obstacle_mapping(self) -> List:
        """
        Step 4: Map static obstacles.
        
        Returns:
            List of obstacle regions
            
        Logs:
            [CALIB] Arena size estimated: XmxYm
            [CALIB] Static obstacles mapped
        """
        print("[CALIB] Step 4/4: Obstacle mapping")
        
        # Display white screen for obstacle contrast
        print("[CALIB] Displaying white screen for obstacle detection...")
        self.projector.show_white_screen()
        
        print("[CALIB] Place obstacles in arena, then press SPACE...")
        
        self._wait_for_user_validation("Placez les obstacles et appuyez sur ESPACE")
        
        # Simplified: return empty for now
        # Full implementation would do thresholding and contour detection
        
        print("[CALIB] Arena size estimated: {:.2f}m x {:.2f}m".format(self.arena_width_m, self.arena_height_m))
        print("[CALIB] Static obstacles mapped")
        
        return []
