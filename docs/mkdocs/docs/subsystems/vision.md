# Vision Subsystem

The vision subsystem uses PhotonVision for game element detection and tracking. This system provides robust object detection capabilities for autonomous and teleoperated modes.

## Game Element Detection

The `GameElement` class provides a framework for defining and tracking game pieces with the following properties:
- Name identification
- Physical dimensions (length, width, height in meters)
- Position tracking

## Features

- Real-time game element tracking
- Dimension-based filtering
- Integration with PhotonVision pipeline

## Usage Example

```java
GameElement gamePiece = new GameElement("Cube", 0.3, 0.3, 0.3);
// Use the game piece for vision tracking
```

## Best Practices

1. Calibrate cameras before each competition
2. Verify physical measurements of game elements
3. Test vision system under various lighting conditions
