# Bling Subsystem

## Overview
The BlingSubsystem controls LED lighting (Bling) on the robot using a CTRE CANdle controller. This subsystem provides comprehensive LED control capabilities, including solid colors, animations, queued patterns, and alliance-based color schemes. The system is designed to provide visual feedback about robot state and enhance field presence.

## Core Features
The subsystem provides robust LED control through several key mechanisms:

### Color Management
The system supports immediate and queued color changes:
```java
// Set immediate color
bling.setSolidColor(Color.RED);

// Queue next color
bling.queueColor(nextColor);
bling.setQueuedColor();  // Apply when ready
```

### Animation Control
Animations can be controlled directly or through a queue system:
```java
// Run animation immediately
bling.runAnimation(new RainbowAnimation());

// Queue animation
bling.queueAnimation(nextAnimation);
bling.runQueuedAnimation();  // Apply when ready
```

### Alliance Integration
The system automatically handles alliance colors:
```java
bling.setLedToAllianceColor();  // Sets red/blue based on alliance
```

## Configuration

### Hardware Setup
The system expects:
- CTRE CANdle controller
- GRB LED strip type
- Configurable number of LEDs (default 92)
- Optional LED offset (default 8)

### Tunable Constants
Key parameters in BlingSubsystemConstants:
```java
public static final boolean BLING_ENABLED = false;
public static final double MAX_LED_BRIGHTNESS = 0.25;
public static final int NUM_LEDS = 92;
public static final int LED_OFFSET = 8;
```

## Telemetry
The subsystem includes comprehensive telemetry through Shuffleboard:
- Current color display
- Active animation status
- System enable/disable status

## Best Practices

### State Management
The subsystem maintains clear state through dedicated methods:
- `clearAnimation()` for stopping animations
- `clearSolidColor()` for turning off LEDs
- `clearAll()` for complete reset
- `runDefault()` for returning to baseline behavior

### Update Cycle
The subsystem handles updates automatically through:
- Periodic updates in the subsystem
- Automatic telemetry updates
- State management in the command loop

## Integration Notes
To add this subsystem to your robot:

1. Configure CAN ID for the CANdle
2. Adjust constants for your LED strip configuration
3. Set up default command (typically DefaultSetToAllianceColor)
4. Integrate with robot state indicators as needed

This subsystem is designed to be both robust for competition use and flexible for development and testing purposes.
