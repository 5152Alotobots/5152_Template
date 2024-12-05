# Drivetrain Subsystem

The drivetrain subsystem implements a Swerve drive system using CTRE motor controllers. This system provides precise control over robot movement with features like:

## Features

- Deadband handling for precise control
- Telemetry reporting via Shuffleboard
- Field-oriented driving capabilities

## Key Components

### Telemetry

The `SwerveDriveTelemetry` class provides real-time feedback including:
- Robot pose (X, Y position and rotation)
- Module states
- Field visualization

### Usage Example

```java
// Example of using deadband controls
double processedInput = JoystickUtilities.joyDeadBnd(rawInput, HMIDeadbands.DRIVER_FWD_AXIS_DEADBAND);
```

## Configuration

The drivetrain uses several deadband constants for smooth operation:
- Forward/Backward: 0.1
- Strafe: 0.1
- Rotation: (see HMIDeadbands class for current value)
