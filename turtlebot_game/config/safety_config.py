# config/safety_config.py
"""Configuration centralisée pour le Safety Bridge"""

class SafetyBridgeConfig:
    """Configuration du WebSocket Safety Bridge"""
    
    # ===== WEBSOCKET =====
    ROBOT_NAMESPACE = "robot1"
    WS_HOST = "0.0.0.0"
    WS_PORT = 8765
    
    # ===== LIMITES DE VITESSE =====
    MAX_LINEAR_VEL = 0.22     # m/s - Vitesse linéaire max débridée
    MIN_LINEAR_VEL = -0.3     # m/s
    MAX_ANGULAR_VEL = 2.84    # rad/s - Vitesse angulaire réelle max du TurtleBot
    
    # ===== SÉCURITÉ (pour version avec détection d'obstacles) =====
    SAFETY_DISTANCE = 0.3           # Distance minimale obstacles (m)
    ROBOT_SAFETY_DISTANCE = 0.5     # Distance minimale entre robots (m)
    ROBOT_RADIUS = 0.2              # Rayon du robot (m)
    
    # ===== PRÉDICTION DE TRAJECTOIRE =====
    PREDICTION_STEPS = 10     # Nombre de steps de prédiction
    PREDICTION_DT = 0.1       # Delta temps entre steps (s)
    
    # ===== LOGGING & DEBUG =====
    LOG_LEVEL = "INFO"        # DEBUG, INFO, WARN, ERROR
    DEBUG = False             # Mode debug verbeux
    
    # ===== QoS ROS2 =====
    QOS_DEPTH = 10
    QOS_RELIABILITY = "RELIABLE"  # RELIABLE ou BEST_EFFORT
    
    # ===== WEBSOCKET AVANCÉ =====
    WS_PING_INTERVAL = 20     # Intervalle ping WebSocket (s)
    WS_PING_TIMEOUT = 10      # Timeout ping (s)
    WS_MAX_MESSAGE_SIZE = 1048576  # 1MB
    
    # ===== BROADCAST =====
    STATUS_BROADCAST_INTERVAL = 5.0  # Intervalle broadcast status (s)
    
    # ===== TIMEOUTS =====
    CMD_TIMEOUT = 1.0         # Timeout commande (s)
    CONNECTION_TIMEOUT = 30.0 # Timeout connexion (s)
    
    @classmethod
    def to_dict(cls):
        """Convertit la config en dictionnaire"""
        return {
            key.lower(): value 
            for key, value in vars(cls).items() 
            if not key.startswith('_') and key.isupper()
        }
    
    @classmethod
    def get_ros_params(cls):
        """Retourne les paramètres au format ROS2"""
        return {
            'robot_namespace': cls.ROBOT_NAMESPACE,
            'ws_host': cls.WS_HOST,
            'ws_port': cls.WS_PORT,
            'max_linear_vel': cls.MAX_LINEAR_VEL,
            'min_linear_vel': cls.MIN_LINEAR_VEL,
            'max_angular_vel': cls.MAX_ANGULAR_VEL,
            'safety_distance': cls.SAFETY_DISTANCE,
            'robot_safety_distance': cls.ROBOT_SAFETY_DISTANCE,
            'robot_radius': cls.ROBOT_RADIUS,
            'prediction_steps': cls.PREDICTION_STEPS,
            'prediction_dt': cls.PREDICTION_DT,
            'log_level': cls.LOG_LEVEL,
            'debug': cls.DEBUG,
            'qos_depth': cls.QOS_DEPTH,
            'ws_ping_interval': cls.WS_PING_INTERVAL,
            'ws_ping_timeout': cls.WS_PING_TIMEOUT,
            'status_broadcast_interval': cls.STATUS_BROADCAST_INTERVAL,
            'cmd_timeout': cls.CMD_TIMEOUT,
        }


# Configurations pré-définies
class DevConfig(SafetyBridgeConfig):
    """Configuration pour développement"""
    DEBUG = True
    LOG_LEVEL = "DEBUG"
    STATUS_BROADCAST_INTERVAL = 2.0


class ProdConfig(SafetyBridgeConfig):
    """Configuration pour production"""
    DEBUG = False
    LOG_LEVEL = "INFO"
    MAX_LINEAR_VEL = 0.22  # Vitesse débridée (anciennement 0.3)


class FastConfig(SafetyBridgeConfig):
    """Configuration pour robot rapide"""
    MAX_LINEAR_VEL = 0.22   # Limité à la vitesse max réelle du TurtleBot
    MAX_ANGULAR_VEL = 2.84  # Vitesse angulaire réelle max du TurtleBot