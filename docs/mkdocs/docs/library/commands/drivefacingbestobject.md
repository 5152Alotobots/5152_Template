# DriveFacingBestObject Command

## Purpose
The DriveFacingBestObject command enables field-centric robot driving while automatically orienting the robot to face detected game pieces. It combines manual translation control with automated rotation management, providing smooth transitions between automated facing and manual override capabilities.

This command works in conjunction with the [PhotonVision Object Detection System](../subsystems/vision/objectdetection.md) and [SwerveDrive Subsystem](../subsystems/swerve.md).

See also: [PathfindToBestObject](./pathfindtobestobject.md) for autonomous navigation to objects.

## Required Subsystems
- [PhotonVision Object Detection System](../subsystems/vision/objectdetection.md)
- [SwerveDrive Subsystem](../subsystems/swerve.md)

## Constructor
```java
public DriveFacingBestObject(
    PhotonVisionObjectDetectionSubsystem objectDetectionSubsystem,
    SwerveDriveSubsystem swerveDriveSubsystem,
    String targetGameElementName,
    DoubleSupplier velocityX,
    DoubleSupplier velocityY,
    DoubleSupplier velocityRotation
)
```

The constructor accepts suppliers for velocity control, allowing flexible input sources such as joysticks or programmatic values. The velocity inputs control field-centric movement while the command manages rotation automatically.

## Operation Modes

### Object Detected Mode
When objects are detected, the command:
- Maintains field-centric translation using manual X/Y inputs
- Automatically rotates to face the highest-confidence detected object
- Uses a PID controller (P=5.0) for rotation control

### No Detection Mode
When no objects are detected, the command:
- Maintains field-centric translation using manual X/Y inputs
- Allows manual rotation control through the velocityRotation supplier
- Operates as a standard field-centric drive command

## Manual Override
The command features a brief manual override capability:
- Override activates when manual rotation input is detected
- Override timeout of 0.1 seconds
- Command ends when override timeout elapses

## Configuration
Key parameters that may need tuning:
```java
// Rotation control PID values
driveFacingAngle.HeadingController = new PhoenixPIDController(5.0, 0, 0.0);

// Override timeout duration
private static final double OVERRIDE_TIMEOUT_SECONDS = 0.1;
```

## Usage Example
```java
new DriveFacingBestObject(
    visionSubsystem,
    driveSubsystem,
    () -> -driverController.getLeftY(),   // Forward/back
    () -> -driverController.getLeftX(),   // Left/right
    () -> -driverController.getRightX()   // Manual rotation override
);
```

This command is particularly useful for game piece acquisition sequences where the driver needs to approach detected objects while maintaining optimal orientation.
