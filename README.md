# VA-51 Tank Arena

Système de jeu en réalité augmentée opposant deux robots TurtleBot dans une arène physique.

## Architecture

```
VA-51/
├── tank_project/        # Jeu principal (IA, Vision, Game Engine)
├── control_turtlebot/   # Interface de contrôle
└── turtlebot_game/      # ROS2 Bridge
```

## Prérequis

- ROS2 Humble
- Python 3.10+
- Intel RealSense SDK
- Pygame

## Communication

Tous les composants communiquent sur **port 8765** :

```
┌─────────────────┐     ┌─────────────────┐
│  Tank Project   │     │ Control Turtle  │
│   (IA Robot)    │     │   (Manuel)      │
└────────┬────────┘     └────────┬────────┘
         │                       │
         │    WebSocket:8765     │
         └───────────┬───────────┘
                     │
            ┌────────▼────────┐
            │  Safety Bridge  │
            │   (ROS2 Node)   │
            └────────┬────────┘
                     │
            ┌────────▼────────┐
            │    TurtleBot    │
            │    /cmd_vel     │
            └─────────────────┘
```

## Protocole WebSocket

```json
{
  "type": "cmd_vel",
  "linear_x": 0.2,
  "angular_z": 0.5,
  "timestamp": 1234567890.123
}
```

## Documentation

- [Documentation Principale](DOCUMENTATION.md)
- [Tank Project](tank_project/README.md)
- [Control TurtleBot](control_turtlebot/README.md)
- [TurtleBot Game](turtlebot_game/DOCUMENTATION.md)
