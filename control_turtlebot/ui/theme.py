"""Gestion du thème UI pour un style professionnel."""

import pygame
from dataclasses import dataclass
from typing import Tuple, Dict
from pathlib  import Path
import logging

logger = logging.getLogger(__name__)


@dataclass
class ColorScheme:
    """Schéma de couleurs professionnel."""
    background: Tuple[int, int, int]
    panel: Tuple[int, int, int]
    panel_accent: Tuple[int, int, int]
    text: Tuple[int, int, int]
    text_dim: Tuple[int, int, int]
    text_bright: Tuple[int, int, int]
    accent: Tuple[int, int, int]
    accent_bright: Tuple[int, int, int]
    accent_dim: Tuple[int, int, int]
    success: Tuple[int, int, int]
    warning: Tuple[int, int, int]
    error: Tuple[int, int, int]
    info: Tuple[int, int, int]
    grid: Tuple[int, int, int]


class Theme:
    """
    Thème UI avec couleurs et polices.
    
    Fournit un style cohérent à travers l'application.
    """
    
    def __init__(self, config: Dict):
        """
        Initialise le thème depuis la configuration.
        
        Args:
            config: Dictionnaire de configuration UI
        """
        theme_cfg = config.get('theme', {})
        font_cfg = config.get('fonts', {})
        
        # Create color scheme from config
        colors_dict = theme_cfg.get('colors', {})
        self.colors = ColorScheme(
            background=tuple(colors_dict.get('background', [15, 15, 20])),
            panel=tuple(colors_dict.get('panel', [25, 28, 35])),
            panel_accent=tuple(colors_dict.get('panel_accent', [35, 40, 50])),
            text=tuple(colors_dict.get('text', [230, 230, 235])),
            text_dim=tuple(colors_dict.get('text_dim', [150, 150, 160])),
            text_bright=tuple(colors_dict.get('text_bright', [255, 255, 255])),
            accent=tuple(colors_dict.get('accent', [0, 180, 220])),
            accent_bright=tuple(colors_dict.get('accent_bright', [80, 200, 240])),
            accent_dim=tuple(colors_dict.get('accent_dim', [0, 140, 180])),
            success=tuple(colors_dict.get('success', [50, 200, 100])),
            warning=tuple(colors_dict.get('warning', [200, 150, 0])),
            error=tuple(colors_dict.get('error', [200, 50, 50])),
            info=tuple(colors_dict.get('info', [120, 150, 255])),
            grid=tuple(colors_dict.get('grid', [35, 38, 45]))
        )
        
        theme_cfg = config.get('theme', {})
        font_cfg = config.get('fonts', {})
        
        # Crée le schéma de couleurs depuis la config
        self.fonts = self._load_fonts(font_cfg)
        
        logger.info(f"Theme loaded: {theme_cfg.get('name', 'unnamed')}")
    
    def _load_fonts(self, font_cfg: Dict) -> Dict[str, pygame.font.Font]:
        """
        Load fonts with fallback to default.
        
        Args:
            font_cfg: Font configuration dictionary
            
        Returns:
            Dictionary of font name to pygame.font.Font
        """
        fonts = {}
        
        # Font sizes from config (already english code keywords)
        sizes = {
            'title': font_cfg.get('title_size', 32),
            'subtitle': font_cfg.get('subtitle_size', 24),
            'normal': font_cfg.get('normal_size', 20),
            'small': font_cfg.get('small_size', 16),
            'tiny': font_cfg.get('tiny_size', 14)
        }
        
        # Essaie de charger police perso, repli sur defaut
        font_path = font_cfg.get('font_file', None)
        
        for name, size in sizes.items():
            try:
                if font_path and Path(font_path).exists():
                    fonts[name] = pygame.font.Font(font_path, size)
                else:
                    fonts[name] = pygame.font.SysFont('Arial', size, bold=(name == 'title'))
            except Exception as e:
                logger.warning(f"Échec chargement police '{name}' : {e}")
                fonts[name] = pygame.font.SysFont(None, size)
        
        return fonts
    
    def get_font(self, size: str = 'normal') -> pygame.font.Font:
        """
        Obtient la police par nom de taille.
        
        Args:
            size: Nom taille police ('title', 'subtitle', 'normal', 'small', 'tiny')
            
        Returns:
            Objet police Pygame
        """
        return self.fonts.get(size, self.fonts['normal'])
    
    def draw_rounded_rect(self, surface: pygame.Surface, rect: pygame.Rect, 
                         color: Tuple[int, int, int], radius: int = 12):
        """
        Draw a rounded rectangle.
        
        Args:
            surface: Surface to draw on
            rect: Rectangle to draw
            color: Fill color
            radius: Corner radius in pixels
        """
        pygame.draw.rect(surface, color, rect, border_radius=radius)
    
    def draw_panel(self, surface: pygame.Surface, rect: pygame.Rect, 
                  accent: bool = False, radius: int = 12):
        """
        Dessine un panneau stylisé.
        
        Args:
            surface: Surface sur laquelle dessiner
            rect: Rectangle panneau
            accent: Si True, utilise couleur accent
            radius: Rayon coins
        """
        color = self.colors.panel_accent if accent else self.colors.panel
        self.draw_rounded_rect(surface, rect, color, radius)
    
    def render_text(self, text: str, font_size: str = 'normal', 
                    color: Tuple[int, int, int] = None) -> pygame.Surface:
        """
        Rend le texte avec la police du thème.
        
        Args:
            text: Texte à rendre
            font_size: Nom taille police
            color: Couleur texte (défaut: couleur texte thème)
            
        Returns:
            Surface texte rendu
        """
        if color is None:
            color = self.colors.text
        
        font = self.get_font(font_size)
        return font.render(text, True, color)
