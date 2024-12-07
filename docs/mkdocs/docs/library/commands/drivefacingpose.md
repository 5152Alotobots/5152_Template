# DriveFacingPose Command

## Overview

The `DriveFacingPose` command enables the robot to drive while automatically maintaining its orientation towards a specified pose on the field. This is useful for scenarios where you want the robot to keep facing a particular location while moving around, such as:

- Keeping the robot's intake facing a game piece collection station
- Maintaining visual focus on a scoring location while maneuvering
- Following a path while keeping orientation towards a specific field element

## Usage

### Constructor

```java
public DriveFacingPose(
    Pose2d targetPose,
    SwerveDriveSubsystem swerveDriveSubsystem,
    DoubleSupplier velocityX,
    DoubleSupplier velocityY,
    DoubleSupplier velocityRotation)
```

Parameters:
- `targetPose`: The field-relative pose that the robot should face
- `swerveDriveSubsystem`: The swerve drive subsystem for controlling robot movement
- `velocityX`: Supplier for forward/backward velocity (-1.0 to 1.0)
- `velocityY`: Supplier for left/right velocity (-1.0 to 1.0)
- `velocityRotation`: Supplier for manual rotation override (-1.0 to 1.0)

### Example

```java
// Create a command to drive while facing the blue alliance scoring station
Pose2d blueStation = new Pose2d(1.0, 5.5, new Rotation2d());
DriveFacingPose driveFacingCommand = new DriveFacingPose(
    blueStation,
    m_driveSubsystem,
    () -> -m_driverController.getLeftY(),   // Forward/back from left stick
    () -> -m_driverController.getLeftX(),   // Left/right from left stick
    () -> -m_driverController.getRightX()   // Manual rotation from right stick
);
```

## Behavior

The command operates in two modes:

### Auto-Facing Mode
When no manual rotation input is detected (rotation stick near neutral):
- Calculates angle to target pose using field-relative coordinates
- Uses PID control to maintain robot orientation towards target
- Allows full translation movement while maintaining target facing

### Manual Override Mode
When manual rotation input is detected:
- Switches to full manual control
- Allows driver to temporarily override the facing direction
- Returns to auto-facing mode after a brief timeout when manual input stops

## Configuration

The command uses these key configuration values:

- Rotation PID: P=5.0, I=0.0, D=0.0
- Manual control threshold: 0.1 (10% stick movement)
- Override timeout: 0.1 seconds

## Requirements

- CTRE Phoenix 6 library
- A properly configured SwerveDriveSubsystem
- Field-relative odometry must be calibrated
