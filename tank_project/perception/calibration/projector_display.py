"""
Projector Display for Calibration - FINAL VERSION

Displays ArUco markers on projector for calibration wizard.
Features:
- Automatic positioning on secondary screen (VGA/HDMI)
- Borderless mode to hide window decorations
- Hidden cursor for immersion

Logs: [PROJ_DISPLAY] prefix
"""

import os
import pygame
import cv2
import numpy as np
from typing import Tuple, Optional


class ProjectorDisplay:
    """
    Pygame window for projecting calibration patterns.
    Forces display on secondary monitor using SDL environment variables.
    """
    
    def __init__(self, width=1024, height=768, margin=50, monitor_offset_x=1920, monitor_offset_y=0,
                 borderless=True, hide_cursor=True):
        """
        Initialize projector display.
        
        Args:
            width: Projector resolution width (ex: 1024 for VGA)
            height: Projector resolution height (ex: 768 for VGA)
            margin: Safety margin from edges (px)
            monitor_offset_x: X position of the projector (usually width of main screen, e.g. 1920)
            monitor_offset_y: Y position (usually 0)
            borderless: Use borderless window mode (NOFRAME)
            hide_cursor: Hide mouse cursor
        """
        self.width = width
        self.height = height
        self.margin = margin
        
        # --- CONFIGURATION MULTI-ECRAN ---
        self.monitor_x = monitor_offset_x
        self.monitor_y = monitor_offset_y
        self.borderless = borderless
        self.hide_cursor = hide_cursor
        
        self.screen = None
        self.running = False
        
        # Arena rectangle (with margins)
        self.arena_x1 = margin
        self.arena_y1 = margin
        self.arena_x2 = width - margin
        self.arena_y2 = height - margin
        self.arena_w = self.arena_x2 - self.arena_x1
        self.arena_h = self.arena_y2 - self.arena_y1
        
        print("[PROJ_DISPLAY] Init: {}x{}, margin={}px".format(width, height, margin))
        print("[PROJ_DISPLAY] Target Monitor Offset: X={}, Y={}".format(self.monitor_x, self.monitor_y))
    
    def start(self):
        """Start Pygame and create window on the projector."""
        
        # 1. LE HACK: On force la position avant l'init de l'écran
        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (self.monitor_x, self.monitor_y)
        
        pygame.init()
        
        # 2. Configure window flags based on settings
        flags = pygame.DOUBLEBUF
        if self.borderless:
            flags |= pygame.NOFRAME
        
        self.screen = pygame.display.set_mode((self.width, self.height), flags)
        
        # 3. IMMERSION - Hide cursor if configured
        if self.hide_cursor:
            pygame.mouse.set_visible(False)
        
        pygame.display.set_caption("Tank Arena - Projector View")
        self.running = True
        
        mode_str = "borderless" if self.borderless else "windowed"
        print(f"[PROJ_DISPLAY] Window opened at offset {self.monitor_x} ({mode_str})")
    
    def stop(self):
        """Close display."""
        if self.running:
            pygame.quit()
            self.running = False
            print("[PROJ_DISPLAY] Display closed")
    
    def clear(self, color=(0, 0, 0)):
        """Clear screen to solid color."""
        if self.screen:
            self.screen.fill(color)
    
    def get_events(self):
        """
        Return events to external controller (Wizard).
        Use this instead of handle_events when logic is controlled outside.
        """
        if not self.running:
            return []
        return pygame.event.get()

    def show_corner_markers(self, marker_size_px=200, aruco_dict=cv2.aruco.DICT_4X4_100):
        """
        Display ArUco markers at 4 corners of arena.
        """
        if not self.running:
            return
        
        # Clear to white background
        self.clear((255, 255, 255))
        
        # Generate markers
        aruco_dict_obj = cv2.aruco.getPredefinedDictionary(aruco_dict)
        
        # Corner positions (center of marker)
        corners = {
            0: (self.arena_x1 + marker_size_px // 2, self.arena_y2 - marker_size_px // 2),  # Bottom-left
            1: (self.arena_x2 - marker_size_px // 2, self.arena_y2 - marker_size_px // 2),  # Bottom-right
            2: (self.arena_x2 - marker_size_px // 2, self.arena_y1 + marker_size_px // 2),  # Top-right
            3: (self.arena_x1 + marker_size_px // 2, self.arena_y1 + marker_size_px // 2),  # Top-left
        }
        
        print("[PROJ_DISPLAY] Projecting 4 corner markers (IDs 0-3)")
        
        for marker_id, (cx, cy) in corners.items():
            # Generate marker image
            marker_img = cv2.aruco.generateImageMarker(aruco_dict_obj, marker_id, marker_size_px)
            
            # Convert to pygame surface
            marker_img_rgb = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2RGB)
            marker_surface = pygame.surfarray.make_surface(
                np.transpose(marker_img_rgb, (1, 0, 2))
            )
            
            # Calculate top-left corner
            x = cx - marker_size_px // 2
            y = cy - marker_size_px // 2
            
            # Draw marker
            self.screen.blit(marker_surface, (x, y))
            
            # Add ID label below marker
            font = pygame.font.Font(None, 36)
            text = font.render(f"ID {marker_id}", True, (0, 0, 0))
            text_rect = text.get_rect(center=(cx, cy + marker_size_px // 2 + 30))
            self.screen.blit(text, text_rect)
        
        # Update display
        pygame.display.flip()
        
        print("[PROJ_DISPLAY] Corner markers displayed")
    
    def show_white_screen(self):
        """Display solid white screen (for obstacle detection)."""
        if not self.running:
            return
        
        self.clear((255, 255, 255))
        
        # Add text instruction
        font = pygame.font.Font(None, 48)
        text = font.render("Place obstacles in arena", True, (0, 0, 0))
        text_rect = text.get_rect(center=(self.width // 2, 100))
        self.screen.blit(text, text_rect)
        
        pygame.display.flip()
        print("[PROJ_DISPLAY] White screen displayed")
    
    def show_message(self, message: str, color=(255, 255, 255), bg_color=(0, 0, 0)):
        """
        Display a text message.
        """
        if not self.running:
            return
        
        self.clear(bg_color)
        
        font = pygame.font.Font(None, 72)
        text = font.render(message, True, color)
        text_rect = text.get_rect(center=(self.width // 2, self.height // 2))
        self.screen.blit(text, text_rect)
        
        pygame.display.flip()
    
    def _pump_events(self):
        """
        Internal: Keep window responsive without consuming events.
        Call this in long operations if not using get_events().
        """
        pygame.event.pump()
