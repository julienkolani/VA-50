"""
ROS Bridge Client - Communication with ROS System

WebSocket client for sending commands to ROS bridge:
- Connects to ROS bridge server via WebSocket
- Sends velocity commands to /cmd_vel
- Receives acknowledgments
- Maintains connection health

Logs: [ROS] Command sent: v=X, ω=Y
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
    print("[ROS] Warning: websockets not installed, using fallback")


class ROSBridgeClient:
    """
    Client for communicating with ROS bridge via WebSocket.
    
    Sends velocity commands to physical robot.
    """
    
    def __init__(self, host: str = 'localhost', port: int = 8765):
        """
        Initialize ROS bridge client.
        
        Args:
            host: ROS bridge server host
            port: ROS bridge server port (default 8765 for WebSocket)
        """
        self.host = host
        self.port = port
        self.ws = None
        self.connected = False
        self.uri = f"ws://{host}:{port}"
        
    def connect(self, max_retries: int = 3, retry_interval: float = 2.0):
        """
        Establish WebSocket connection to ROS bridge.
        
        Args:
            max_retries: Max retry attempts
            retry_interval: Seconds between retries
        
        Logs:
            [ROS] Connected to bridge at host:port
            [ROS] Connection failed: error
        """
        if not HAS_WEBSOCKETS:
            print("[ROS] WebSocket library not available")
            return False
            
        attempt = 0
        while attempt < max_retries:
            attempt += 1
            try:
                self.ws = ws_connect(self.uri, open_timeout=5)
                self.connected = True
                
                # Read welcome message
                try:
                    welcome = self.ws.recv(timeout=2)
                    print(f"[ROS] Connected to bridge at {self.host}:{self.port}")
                except:
                    print(f"[ROS] Connected to bridge at {self.host}:{self.port}")
                    
                return True
                
            except Exception as e:
                self.connected = False
                print(f"[ROS] Connection attempt {attempt} failed: {e}")
                
                if attempt >= max_retries:
                    print("[ROS] Max retries reached, running without ROS connection")
                    return False
                
                print(f"[ROS] Retrying in {retry_interval} seconds...")
                time.sleep(retry_interval)
        
        return False
    
    def disconnect(self):
        """Close WebSocket connection to ROS bridge."""
        if self.ws:
            try:
                self.ws.close()
            except:
                pass
            self.connected = False
            print("[ROS] Disconnected from bridge")
    
    def send_velocity_command(self, 
                             robot_id: int, 
                             v: float, 
                             omega: float) -> bool:
        """
        Send velocity command to robot.
        
        Args:
            robot_id: 4 (AI) or 5 (Human)
            v: Linear velocity in m/s
            omega: Angular velocity in rad/s
            
        Returns:
            True if sent successfully
            
        Message format (JSON):
            {
                "type": "cmd_vel",
                "linear_x": v,
                "angular_z": omega,
                "timestamp": unix_time
            }
            
        Logs:
            [ROS] Robot4 cmd: v=0.15 m/s, ω=-0.30 rad/s
        """
        if not self.connected or not self.ws:
            # Try to reconnect silently
            if not self.connect(max_retries=1, retry_interval=0.5):
                return False
        
        # Message format matching safety_bridge.py expectations
        message = {
            "type": "cmd_vel",
            "linear_x": round(v, 4),
            "angular_z": round(omega, 4),
            "timestamp": time.time()
        }
        
        try:
            msg_json = json.dumps(message)
            self.ws.send(msg_json)
            
            # Log (less verbose - only log every 30th command or significant ones)
            if abs(v) > 0.01 or abs(omega) > 0.1:
                print(f"[ROS] Robot{robot_id} cmd: v={v:.2f} m/s, w={omega:.2f} rad/s")
            
            # Try to receive acknowledgment (non-blocking)
            try:
                self.ws.recv(timeout=0.01)
            except:
                pass  # Ignore if no response
            
            return True
            
        except Exception as e:
            print(f"[ROS] Send failed: {e}")
            self.connected = False
            return False
    
    def receive_feedback(self, timeout: float = 0.01) -> Optional[dict]:
        """
        Receive feedback from ROS bridge (non-blocking).
        
        Args:
            timeout: Socket timeout in seconds
            
        Returns:
            dict with data, or None
        """
        if not self.connected or not self.ws:
            return None
        
        try:
            data = self.ws.recv(timeout=timeout)
            if data:
                return json.loads(data)
        except:
            pass  # No data available
            
        return None
    
    def send_stop_command(self, robot_id: int):
        """
        Send emergency stop to robot.
        
        Args:
            robot_id: Robot to stop
        """
        self.send_velocity_command(robot_id, 0.0, 0.0)
