#!/usr/bin/env python3
"""
Configuration globale du TurtleBot Controller
Tous les paramètres modifiables de l'application
"""

# ============================================================================
# CONNEXION WEBSOCKET
# ============================================================================
WEBSOCKET_URI = "ws://localhost:8765"
WEBSOCKET_RECONNECT_DELAY = 2.0  # secondes
WEBSOCKET_PING_INTERVAL = 3.0    # secondes

# ============================================================================
# AMPLIFICATION DES COMMANDES ROBOT
# ============================================================================
# Facteurs d'amplification envoyés au robot via WebSocket
# Augmentez ces valeurs pour un robot plus réactif
# Réduisez-les pour un comportement plus doux
K_LINEAR = 1.0   # Amplification vitesse linéaire (défaut: 6.0)
K_ANGULAR = 1.0  # Amplification vitesse angulaire (défaut: 6.0)

# ============================================================================
# CONTRÔLEURS
# ============================================================================
# Facteur de vitesse initial (peut être modifié avec +/-)
INITIAL_SPEED_FACTOR = 1.0  # Débloqué à 100% (ancien: 0.07)

# Limites des facteurs de vitesse
MIN_SPEED_FACTOR = 0.2
MAX_SPEED_FACTOR = 2.0
SPEED_INCREMENT = 0.1  # Incrément pour +/-

# Deadzone pour la manette PS3 (évite le drift)
PS3_DEADZONE = 0.15

# Fréquence d'envoi des commandes (Hz)
COMMAND_FREQUENCY = 30

# ============================================================================
# CLAVIER - Paramètres de contrôle
# ============================================================================
KEYBOARD_ACCEL_LINEAR = 0.2      # Accélération linéaire
KEYBOARD_ACCEL_ANGULAR = 2.5     # Accélération angulaire
KEYBOARD_MAX_LINEAR = 3.5        # Vitesse linéaire maximale
KEYBOARD_MAX_ANGULAR = 120       # Vitesse angulaire maximale (degrés)
KEYBOARD_FRICTION = 0.92         # Friction linéaire (0-1)
KEYBOARD_FRICTION_ANGULAR = 0.82 # Friction angulaire (0-1)

# ============================================================================
# MANETTE PS3 - Paramètres de contrôle
# ============================================================================
PS3_MAX_LINEAR = 3.5    # Vitesse linéaire maximale
PS3_MAX_ANGULAR = 120   # Vitesse angulaire maximale (degrés)

# Mapping des boutons PS3 (peut varier selon le driver)
PS3_BTN_X = 0           # Croix (arrêt d'urgence)
PS3_BTN_CIRCLE = 1      # Rond
PS3_BTN_TRIANGLE = 2    # Triangle
PS3_BTN_SQUARE = 3      # Carré

# Axes analogiques PS3
PS3_AXIS_LEFT_X = 0     # Stick gauche horizontal
PS3_AXIS_LEFT_Y = 1     # Stick gauche vertical
PS3_AXIS_RIGHT_X = 2    # Stick droit horizontal

# ============================================================================
# SIMULATION VISUELLE
# ============================================================================
# Échelles de mouvement dans la simulation visuelle
VISUAL_SPEED_SCALE = 1000      # pixels par m/s
VISUAL_ANGULAR_SCALE = 5       # pixels par rad/s

# Paramètres du robot visuel
VISUAL_MAX_DISTANCE = 220      # Distance max du centre (pixels)
VISUAL_TRAIL_LENGTH = 80       # Longueur de la traînée
VISUAL_RETURN_SPEED = 0.15     # Vitesse de retour au centre

# ============================================================================
# INTERFACE GRAPHIQUE
# ============================================================================
# Dimensions par défaut
DEFAULT_WIDTH = 1200
DEFAULT_HEIGHT = 700
INFO_PANEL_WIDTH = 350

# FPS cible
TARGET_FPS = 60

# Fréquence de mise à jour du status (secondes)
STATUS_UPDATE_INTERVAL = 3.0

# ============================================================================
# COULEURS - Thème sombre moderne avec accents néon
# ============================================================================
# Fond et panneaux
COLOR_BG = (15, 15, 20)              # Fond très sombre
COLOR_PANEL = (25, 28, 35)           # Panneaux légèrement plus clairs
COLOR_PANEL_ACCENT = (35, 40, 50)    # Accent pour sous-panneaux

# Texte
COLOR_TEXT = (230, 230, 235)         # Texte principal clair
COLOR_TEXT_DIM = (150, 150, 160)     # Texte secondaire
COLOR_TEXT_BRIGHT = (255, 255, 255)  # Texte en surbrillance

# Accents principaux (néon cyan/bleu)
COLOR_ACCENT = (0, 230, 255)         # Cyan néon principal
COLOR_ACCENT_BRIGHT = (100, 240, 255) # Cyan clair
COLOR_ACCENT_DIM = (0, 180, 200)     # Cyan foncé

# États
COLOR_SUCCESS = (50, 255, 150)       # Vert néon (succès)
COLOR_WARNING = (255, 180, 0)        # Orange vif (attention)
COLOR_ERROR = (255, 80, 100)         # Rouge néon (erreur)
COLOR_INFO = (150, 150, 255)         # Bleu clair (info)

# Grille et éléments de fond
COLOR_GRID = (35, 38, 45)            # Grille subtile
COLOR_GRID_ACCENT = (45, 50, 60)     # Lignes de grille accentuées

# Robot visuel
COLOR_ROBOT_BODY = (40, 45, 55)      # Corps du robot
COLOR_ROBOT_OUTLINE = (0, 220, 255)  # Contour cyan néon
COLOR_ROBOT_WHEELS = (90, 95, 105)   # Roues
COLOR_ROBOT_DIRECTION = (255, 100, 120) # Flèche de direction (rose néon)
COLOR_ROBOT_CENTER = (255, 220, 0)   # Point central (jaune)

# Traînée du robot (gradient cyan)
COLOR_TRAIL_START = (0, 100, 120)    # Début de traînée
COLOR_TRAIL_END = (0, 200, 220)      # Fin de traînée

# Effets visuels
COLOR_GLOW = (0, 200, 255, 100)      # Effet de lueur (avec alpha)
COLOR_SHADOW = (0, 0, 0, 80)         # Ombre portée

# ============================================================================
# POLICES
# ============================================================================
FONT_SIZE_TITLE = 36        # Titres principaux
FONT_SIZE_SUBTITLE = 28     # Sous-titres
FONT_SIZE_NORMAL = 24       # Texte normal
FONT_SIZE_SMALL = 20        # Petit texte
FONT_SIZE_TINY = 16         # Texte minuscule

# ============================================================================
# ANIMATIONS
# ============================================================================
ANIMATION_PULSE_SPEED = 2.0       # Vitesse de pulsation (connexion)
ANIMATION_GLOW_INTENSITY = 0.3    # Intensité de la lueur
ANIMATION_SMOOTH_FACTOR = 0.15    # Facteur de lissage des animations

# ============================================================================
# MESSAGES DE DEBUG
# ============================================================================
DEBUG_MODE = False  # Active les messages de debug supplémentaires


def get_websocket_config():
    """Retourne la configuration WebSocket"""
    return {
        'uri': WEBSOCKET_URI,
        'reconnect_delay': WEBSOCKET_RECONNECT_DELAY,
        'ping_interval': WEBSOCKET_PING_INTERVAL,
        'k_linear': K_LINEAR,
        'k_angular': K_ANGULAR
    }


def get_controller_config():
    """Retourne la configuration des contrôleurs"""
    return {
        'initial_speed_factor': INITIAL_SPEED_FACTOR,
        'min_speed_factor': MIN_SPEED_FACTOR,
        'max_speed_factor': MAX_SPEED_FACTOR,
        'speed_increment': SPEED_INCREMENT,
        'command_frequency': COMMAND_FREQUENCY
    }


def get_keyboard_config():
    """Retourne la configuration du contrôleur clavier"""
    return {
        'accel_linear': KEYBOARD_ACCEL_LINEAR,
        'accel_angular': KEYBOARD_ACCEL_ANGULAR,
        'max_linear': KEYBOARD_MAX_LINEAR,
        'max_angular': KEYBOARD_MAX_ANGULAR,
        'friction': KEYBOARD_FRICTION,
        'friction_angular': KEYBOARD_FRICTION_ANGULAR
    }


def get_ps3_config():
    """Retourne la configuration du contrôleur PS3"""
    return {
        'deadzone': PS3_DEADZONE,
        'max_linear': PS3_MAX_LINEAR,
        'max_angular': PS3_MAX_ANGULAR,
        'btn_x': PS3_BTN_X,
        'btn_circle': PS3_BTN_CIRCLE,
        'btn_triangle': PS3_BTN_TRIANGLE,
        'btn_square': PS3_BTN_SQUARE,
        'axis_left_x': PS3_AXIS_LEFT_X,
        'axis_left_y': PS3_AXIS_LEFT_Y,
        'axis_right_x': PS3_AXIS_RIGHT_X
    }


def get_visual_config():
    """Retourne la configuration de la simulation visuelle"""
    return {
        'speed_scale': VISUAL_SPEED_SCALE,
        'angular_scale': VISUAL_ANGULAR_SCALE,
        'max_distance': VISUAL_MAX_DISTANCE,
        'trail_length': VISUAL_TRAIL_LENGTH,
        'return_speed': VISUAL_RETURN_SPEED
    }


def get_ui_config():
    """Retourne la configuration de l'interface"""
    return {
        'default_width': DEFAULT_WIDTH,
        'default_height': DEFAULT_HEIGHT,
        'info_panel_width': INFO_PANEL_WIDTH,
        'target_fps': TARGET_FPS,
        'status_update_interval': STATUS_UPDATE_INTERVAL
    }


def get_color_scheme():
    """Retourne le schéma de couleurs complet"""
    return {
        'bg': COLOR_BG,
        'panel': COLOR_PANEL,
        'panel_accent': COLOR_PANEL_ACCENT,
        'text': COLOR_TEXT,
        'text_dim': COLOR_TEXT_DIM,
        'text_bright': COLOR_TEXT_BRIGHT,
        'accent': COLOR_ACCENT,
        'accent_bright': COLOR_ACCENT_BRIGHT,
        'accent_dim': COLOR_ACCENT_DIM,
        'success': COLOR_SUCCESS,
        'warning': COLOR_WARNING,
        'error': COLOR_ERROR,
        'info': COLOR_INFO,
        'grid': COLOR_GRID,
        'grid_accent': COLOR_GRID_ACCENT,
        'robot_body': COLOR_ROBOT_BODY,
        'robot_outline': COLOR_ROBOT_OUTLINE,
        'robot_wheels': COLOR_ROBOT_WHEELS,
        'robot_direction': COLOR_ROBOT_DIRECTION,
        'robot_center': COLOR_ROBOT_CENTER,
        'trail_start': COLOR_TRAIL_START,
        'trail_end': COLOR_TRAIL_END
    }