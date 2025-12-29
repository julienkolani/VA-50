#!/usr/bin/env python3
"""
Validation Orientation Monde vs Orientation ArUco

Affiche simultanément :
- le repère MONDE (axes fixes)
- l'orientation AVANT de l'ArUco
- permet de vérifier si l'avant ArUco = avant robot réel
"""

import sys
import os
import yaml
import pygame
import numpy as np
import cv2
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.camera.realsense_stream import RealSenseStream
from perception.camera.aruco_detector import ArucoDetector
from core.world.unified_transform import load_calibration

# ===== Couleurs =====
C_BG = (0, 0, 0)
C_WORLD_X = (255, 0, 0)      # Rouge = +X monde
C_WORLD_Y = (0, 255, 0)      # Vert  = +Y monde
C_ARUCO = (0, 200, 255)      # Cyan = ArUco
C_TEXT = (255, 255, 255)

AXIS_LEN_M = 0.30            # 30 cm


def load_projector_config():
    config_dir = Path(__file__).parent.parent / 'config'
    path = config_dir / 'projector.yaml'
    if path.exists():
        with open(path) as f:
            return yaml.safe_load(f)
    return None


def get_aruco_pose_world(detection, transform_mgr):
    """
    Retourne (x, y, theta_world) depuis l'ArUco
    """
    u, v = detection['center']
    theta_pix = detection['orientation']

    dist_px = 30
    u_f = u + dist_px * np.cos(theta_pix)
    v_f = v - dist_px * np.sin(theta_pix)

    x, y = transform_mgr.camera_to_world(u, v)
    x_f, y_f = transform_mgr.camera_to_world(u_f, v_f)

    theta = np.arctan2(y_f - y, x_f - x)
    return x, y, theta


def main():
    print("[CHECK] Monde vs ArUco orientation")

    transform_mgr = load_calibration(str(Path(__file__).parent.parent / "config"))
    if not transform_mgr.is_calibrated():
        print("[ERREUR] Calibration absente")
        return

    proj_conf = load_projector_config()
    if proj_conf:
        win_w = proj_conf['projector']['width']
        win_h = proj_conf['projector']['height']
        off_x = proj_conf['display'].get('monitor_offset_x', 0)
        off_y = proj_conf['display'].get('monitor_offset_y', 0)
    else:
        win_w = transform_mgr.proj_width
        win_h = transform_mgr.proj_height
        off_x, off_y = 1920, 0

    os.environ['SDL_VIDEO_WINDOW_POS'] = f"{off_x},{off_y}"

    pygame.init()
    screen = pygame.display.set_mode((win_w, win_h), pygame.NOFRAME)
    font = pygame.font.SysFont("Consolas", 22, bold=True)

    camera = RealSenseStream(1280, 720, 30)
    camera.start()
    time.sleep(1.0)

    K, D = camera.get_intrinsics_matrix()
    aruco = ArucoDetector()

    clock = pygame.time.Clock()
    running = True

    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False

            frame, _ = camera.get_frames()
            if frame is None:
                continue

            if K is not None and D is not None:
                frame = cv2.undistort(frame, K, D)

            detections = aruco.detect(frame)

            screen.fill(C_BG)

            # ===== Repère MONDE fixe =====
            ox = transform_mgr.arena_width_m / 2
            oy = transform_mgr.arena_height_m / 2

            o_px = transform_mgr.world_to_projector(ox, oy)

            # +X monde
            x_px = transform_mgr.world_to_projector(ox + AXIS_LEN_M, oy)
            pygame.draw.line(screen, C_WORLD_X, o_px, x_px, 5)
            screen.blit(font.render("+X MONDE", True, C_WORLD_X), (x_px[0] + 5, x_px[1]))

            # +Y monde
            y_px = transform_mgr.world_to_projector(ox, oy + AXIS_LEN_M)
            pygame.draw.line(screen, C_WORLD_Y, o_px, y_px, 5)
            screen.blit(font.render("+Y MONDE", True, C_WORLD_Y), (y_px[0] + 5, y_px[1]))

            # ===== ArUco =====
            for mid, data in detections.items():
                x, y, theta = get_aruco_pose_world(data, transform_mgr)

                c_px = transform_mgr.world_to_projector(x, y)
                f_px = transform_mgr.world_to_projector(
                    x + AXIS_LEN_M * np.cos(theta),
                    y + AXIS_LEN_M * np.sin(theta)
                )

                pygame.draw.circle(screen, C_ARUCO, c_px, 8, 2)
                pygame.draw.line(screen, C_ARUCO, c_px, f_px, 4)

                txt = font.render(
                    f"ID:{mid}  θ_aruco={np.degrees(theta):+.1f}°",
                    True, C_TEXT
                )
                screen.blit(txt, (c_px[0] + 10, c_px[1] - 20))

            screen.blit(
                font.render("Comparez AVANT ArUco (cyan) avec +X monde (rouge)", True, (0, 255, 0)),
                (20, 20)
            )

            pygame.display.flip()
            clock.tick(60)

    finally:
        camera.stop()
        pygame.quit()


if __name__ == "__main__":
    main()
