#!/usr/bin/env python3
"""
Client WebSocket pour TurtleBot Controller.

Ce module gere la communication bidirectionnelle avec le serveur ROS Bridge
via WebSocket. Il fournit une interface thread-safe avec reconnexion
automatique, gestion de file d'attente pour les commandes, et suivi
des statistiques de connexion (latence, taux de succes).

Classes:
    WebSocketClient: Client WebSocket thread-safe avec reconnexion automatique.
"""

import asyncio
import websockets
import json
import threading
import time
from queue import Queue, Empty
import config


class WebSocketClient:
    """Client WebSocket thread-safe avec reconnexion automatique"""
    
    def __init__(self, uri="ws://localhost:8765"):
        self.uri = uri
        self.websocket = None
        self.connected = False
        self.running = False
        self.send_queue = Queue()
        
        self.last_status = "Déconnecté"
        self.commands_sent = 0
        self.commands_rejected = 0
        self.commands_accepted = 0
        self.safety_info = {}
        
        self.connection_attempts = 0
        self.last_connection_attempt = 0
        self.reconnect_delay = 2.0
        
        self.loop = None
        self.thread = None
        
        self.last_latency = 0
        self.avg_latency = 0
        self.ping_count = 0
    
    def start(self):
        """Demarre le client dans un thread"""
        print(f"[WS] Connexion a {self.uri}...")
        self.running = True
        self.thread = threading.Thread(target=self._run_async, daemon=True)
        self.thread.start()
        
        for i in range(100):
            if self.connected:
                print(f"[WS] Connecte au ROS Bridge!")
                return True
            time.sleep(0.1)
            
            if i % 10 == 9:
                print(f"[WS] Tentative de connexion... ({i//10 + 1}s)")
        
        print("[WS] Delai de connexion depasse")
        return False
    
    def _run_async(self):
        """Thread asyncio"""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        try:
            self.loop.run_until_complete(self._main_loop())
        except Exception as e:
            print(f"[WS] Erreur boucle asyncio: {e}")
        finally:
            self.loop.close()
    
    async def _main_loop(self):
        """Boucle principale avec reconnexion automatique"""
        while self.running:
            try:
                self.connection_attempts += 1
                self.last_connection_attempt = time.time()
                
                if self.connection_attempts > 1:
                    print(f"[WS] Tentative de reconnexion #{self.connection_attempts}...")
                
                async with websockets.connect(
                    self.uri,
                    ping_interval=10,
                    ping_timeout=5,
                    close_timeout=2
                ) as ws:
                    self.websocket = ws
                    self.connected = True
                    self.last_status = "Connecte"
                    
                    if self.connection_attempts > 1:
                        print(f"[WS] Reconnecte!")
                    
                    try:
                        welcome = await asyncio.wait_for(ws.recv(), timeout=5.0)
                        data = json.loads(welcome)
                        robot_name = data.get('robot', 'Inconnu')
                        print(f"[WS] Robot: {robot_name}")
                        
                        config = data.get('config', {})
                        print(f"[WS] Limites: linear=[{config.get('min_linear_vel')}, {config.get('max_linear_vel')}], "
                              f"angular=+/-{config.get('max_angular_vel')}")
                    
                    except asyncio.TimeoutError:
                        print("[WS] Timeout en attente du message d'accueil")
                    
                    await asyncio.gather(
                        self._send_loop(),
                        self._receive_loop(),
                        return_exceptions=True
                    )
            
            except websockets.exceptions.WebSocketException as e:
                self.connected = False
                self.last_status = f"Deconnecte"
                print(f"[WS] Connexion perdue: {e}")
                
                if self.running:
                    await asyncio.sleep(self.reconnect_delay)
            
            except Exception as e:
                self.connected = False
                self.last_status = f"Erreur"
                print(f"[WS] Erreur connexion: {type(e).__name__}: {e}")
                
                if self.running:
                    await asyncio.sleep(self.reconnect_delay)
    
    async def _send_loop(self):
        """Boucle d'envoi des commandes"""
        last_ping = time.time()
        ping_interval = 3.0
        
        while self.connected and self.running:
            try:
                try:
                    msg_type, data = self.send_queue.get_nowait()
                    
                    if msg_type == 'cmd_vel':
                        message = {
                            'type': 'cmd_vel',
                            'linear_x': config.K_LINEAR * data['linear_x'],
                            'angular_z': config.K_ANGULAR * data['angular_z'],
                            'timestamp': time.time()
                        }
                        await self.websocket.send(json.dumps(message))
                        self.commands_sent += 1
                    
                    elif msg_type == 'emergency_stop':
                        message = {
                            'type': 'emergency_stop',
                            'timestamp': time.time()
                        }
                        await self.websocket.send(json.dumps(message))
                        print("[WS] ARRET D'URGENCE envoye!")
                    
                    elif msg_type == 'get_status':
                        message = {
                            'type': 'get_status',
                            'timestamp': time.time()
                        }
                        await self.websocket.send(json.dumps(message))
                
                except Empty:
                    pass
                
                now = time.time()
                if now - last_ping >= ping_interval:
                    try:
                        ping_time = time.time()
                        message = {
                            'type': 'ping',
                            'timestamp': ping_time
                        }
                        await self.websocket.send(json.dumps(message))
                        last_ping = now
                    except:
                        pass
                
                await asyncio.sleep(0.01)
            
            except websockets.exceptions.ConnectionClosed:
                break
            except Exception as e:
                print(f"[WS] Erreur envoi: {e}")
                break
    
    async def _receive_loop(self):
        """Boucle de réception"""
        try:
            async for message in self.websocket:
                try:
                    data = json.loads(message)
                    msg_type = data.get('type')
                    
                    if msg_type == 'cmd_accepted':
                        self.commands_accepted += 1
                        reason = data.get('reason', 'OK')
                        self.last_status = f"Accepte: {reason}"
                    
                    elif msg_type == 'cmd_rejected':
                        self.commands_rejected += 1
                        reason = data.get('reason', 'Bloque')
                        self.last_status = f"Rejete: {reason}"
                        print(f"[WS] Commande rejetee: {reason}")
                    
                    elif msg_type == 'status_broadcast':
                        self.last_status = data.get('last_status', 'Inconnu')
                        self.safety_info = data.get('debug', {})
                    
                    elif msg_type == 'status':
                        self.safety_info = data.get('debug', {})
                    
                    elif msg_type == 'pong':
                        sent_time = data.get('timestamp')
                        if sent_time:
                            try:
                                recv_time = time.time()
                                latency = (recv_time - float(sent_time)) * 1000
                                self.last_latency = latency
                                
                                self.ping_count += 1
                                alpha = 0.3
                                self.avg_latency = alpha * latency + (1 - alpha) * self.avg_latency
                            except:
                                pass
                    
                    elif msg_type == 'emergency_stop_ack':
                        print("[WS] Arret d'urgence confirme par le serveur")
                    
                    elif msg_type == 'error':
                        reason = data.get('reason', 'Inconnu')
                        print(f"[WS] Erreur serveur: {reason}")
                
                except json.JSONDecodeError as e:
                    print(f"[WS] Erreur decodage JSON: {e}")
                except Exception as e:
                    print(f"[WS] Erreur traitement message: {e}")
        
        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            print(f"[WS] Erreur reception: {e}")
    
    def send_cmd_vel(self, linear_x, angular_z):
        """Envoie une commande (thread-safe)"""
        if self.connected:
            try:
                self.send_queue.put_nowait(('cmd_vel', {
                    'linear_x': linear_x,
                    'angular_z': angular_z
                }))
            except:
                pass
    
    def send_emergency_stop(self):
        """Arrêt d'urgence (thread-safe)"""
        try:
            self.send_queue.put_nowait(('emergency_stop', {}))
        except:
            pass
    
    def request_status(self):
        """Demande le status au serveur"""
        if self.connected:
            try:
                self.send_queue.put_nowait(('get_status', {}))
            except:
                pass
    
    def get_stats(self):
        """Retourne les statistiques"""
        success_rate = 0
        if self.commands_sent > 0:
            success_rate = (self.commands_accepted) / self.commands_sent
        
        return {
            'connected': self.connected,
            'status': self.last_status,
            'sent': self.commands_sent,
            'accepted': self.commands_accepted,
            'rejected': self.commands_rejected,
            'success_rate': success_rate,
            'safety_info': self.safety_info,
            'latency': self.last_latency,
            'avg_latency': self.avg_latency,
            'connection_attempts': self.connection_attempts
        }
    
    def stop(self):
        """Arrete le client"""
        print("[WS] Arret du client WebSocket...")
        self.running = False
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)