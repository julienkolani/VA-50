"""
Client Pont ROS - Communication avec le système ROS

Client WebSocket pour envoyer des commandes au pont ROS :
- Se connecte au serveur pont ROS via WebSocket
- Envoie des commandes de vitesse sur /cmd_vel
- Reçoit les acquittements
- Maintaint la santé de la connexion

Logs : [ROS] Commande envoyée : v=X, ω=Y
"""

import json
import time
import asyncio
import threading
from typing import Optional

try:
    import websockets
    from websockets.sync.client import connect as ws_connect
    HAS_WEBSOCKETS = True
except ImportError:
    HAS_WEBSOCKETS = False
    print("[ROS] Attention : websockets non installé, utilisation du repli")


class ROSBridgeClient:
    """
    Client pour communiquer avec le pont ROS via WebSocket.
    
    Envoie des commandes de vitesse au robot physique.
    """
    
    def __init__(self, host: str = 'localhost', port: int = 8765):
        """
        Initialise le client pont ROS.
        
        Args:
            host: Hôte du serveur pont ROS
            port: Port du serveur pont ROS (défaut 8765 pour WebSocket)
        """
        self.host = host
        self.port = port
        self.ws = None
        self.connected = False
        self.uri = f"ws://{host}:{port}"
        
    def connect(self, max_retries: int = 3, retry_interval: float = 2.0):
        """
        Établit la connexion WebSocket avec le pont ROS.
        
        Args:
            max_retries: Tentatives max
            retry_interval: Secondes entre les essais
        
        Logs :
            [ROS] Connecté au pont à host:port
            [ROS] Connexion échouée : erreur
        """
        if not HAS_WEBSOCKETS:
            print("[ROS] Bibliothèque WebSocket non disponible")
            return False
            
        attempt = 0
        while attempt < max_retries:
            attempt += 1
            try:
                # Disable keepalive pings (ping_interval=None) to avoid 1011 errors
                # if the bridge is slow or doesn't support pings correctly.
                self.ws = ws_connect(self.uri, open_timeout=5, ping_interval=None)
                self.connected = True
                
                # Lit le message de bienvenue
                try:
                    welcome = self.ws.recv(timeout=2)
                    print(f"[ROS] Connecté au pont à {self.host}:{self.port}")
                except:
                    print(f"[ROS] Connecté au pont à {self.host}:{self.port}")
                    
                return True
                
            except Exception as e:
                self.connected = False
                print(f"[ROS] Tentative de connexion {attempt} échouée : {e}")
                
                if attempt >= max_retries:
                    print("[ROS] Nb max d'essais atteint, exécution sans connexion ROS")
                    return False
                
                print(f"[ROS] Nouvel essai dans {retry_interval} secondes...")
                time.sleep(retry_interval)
        
        return False
    
    def disconnect(self):
        """Ferme la connexion WebSocket au pont ROS."""
        if self.ws:
            try:
                self.ws.close()
            except:
                pass
            self.connected = False
            print("[ROS] Déconnecté du pont")
    
    def send_velocity_command(self, 
                             robot_id: int, 
                             v: float, 
                             omega: float) -> bool:
        """
        Envoie une commande de vitesse au robot.
        
        Args:
            robot_id: 4 (IA) ou 5 (Humain)
            v: Vitesse linéaire en m/s
            omega: Vitesse angulaire en rad/s
            
        Returns:
            True si envoyé avec succès
            
        Format message (JSON) :
            {
                "type": "cmd_vel",
                "linear_x": v,
                "angular_z": omega,
                "timestamp": unix_time
            }
            
        Logs :
            [ROS] Robot4 cmd : v=0.15 m/s, ω=-0.30 rad/s
        """
        if not self.connected or not self.ws:
            # Essaie de reconnecter silencieusement
            if not self.connect(max_retries=1, retry_interval=0.5):
                return False
        
        # Format message correspondant aux attentes de safety_bridge.py
        message = {
            "type": "cmd_vel",
            "robot_id": robot_id,
            "linear_x": round(v, 4),
            "angular_z": round(omega, 4),
            "timestamp": time.time()
        }
        
        try:
            msg_json = json.dumps(message)
            self.ws.send(msg_json)
            
            # Log (moins verbeux - seulement toutes les 30 commandes ou significatives)
            if abs(v) > 0.01 or abs(omega) > 0.1:
                print(f"[ROS] Robot{robot_id} cmd : v={v:.2f} m/s, w={omega:.2f} rad/s")
            
            # Essaie de recevoir l'acquittement (non-bloquant)
            try:
                self.ws.recv(timeout=0.01)
            except:
                pass  # Ignore si pas de réponse
            
            return True
            
        except Exception as e:
            print(f"[ROS] Envoi échoué : {e}")
            self.connected = False
            return False
    
    def receive_feedback(self, timeout: float = 0.01) -> Optional[dict]:
        """
        Reçoit le retour du pont ROS (non-bloquant).
        
        Args:
            timeout: Timeout socket en secondes
            
        Returns:
            dict avec données, ou None
        """
        if not self.connected or not self.ws:
            return None
        
        try:
            data = self.ws.recv(timeout=timeout)
            if data:
                return json.loads(data)
        except:
            pass  # Pas de données disponibles
            
        return None
    
    def send_stop_command(self, robot_id: int):
        """
        Envoie un arrêt d'urgence au robot.
        
        Args:
            robot_id: Robot à arrêter
        """
        self.send_velocity_command(robot_id, 0.0, 0.0)
