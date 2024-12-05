# SwerveDrive Subsystem

## System Overview

The SwerveDrive system consists of multiple interconnected components that work together to provide a complete swerve drive implementation for FRC robots. The system is built on CTRE's Phoenix 6 framework and integrates with PathPlanner for autonomous path following.

## Core Components

### 1. SwerveDriveSubsystem
The main subsystem class that handles drive control and odometry. This class extends CTRE's `SwerveDrivetrain` and implements WPILib's `Subsystem` interface.

### 2. SwerveDrivePathPlanner
Manages autonomous path following and path generation capabilities. This includes auto path selection and execution.

### 3. TunerConstants
Year-specific configuration files (mk4il22023 and mk4il32024) that contain hardware-specific constants and module configurations.

### 4. SwerveDriveTelemetry
Handles data visualization and debugging through Shuffleboard.

### 5. SwerveDriveSubsystemConstants
Contains performance-related constants for different drive modes.

## Configuration Guide

### Module Configuration

Module configuration is handled in the year-specific TunerConstants files. To configure for your robot:

1. Choose the appropriate year's TunerConstants file:
    - `mk4il22023/TunerConstants.java` for L2 robot (2023)
    - `mk4il32024/TunerConstants.java` for L3 robot (2024)
2. Or, generate a new one via PhoenixTunerX

3. Update the following constants in your chosen TunerConstants file:
```java
// PID Gains
private static final Slot0Configs steerGains = new Slot0Configs()
    .withKP(100)  // Adjust based on your robot's steering response
    .withKI(0)    
    .withKD(0.2);

private static final Slot0Configs driveGains = new Slot0Configs()
    .withKP(3)    // Adjust based on your robot's driving response
    .withKI(0)
    .withKD(0);
```

### PathPlanner Integration

In SwerveDrivePathPlanner.java:

1. Configure path constraints:
```java
// In the corresponding year's TunerConstants.java
public static final PathConstraints PATHFINDING_CONSTRAINTS = new PathConstraints(
    5.2,  // Max velocity (m/s)
    3.5,  // Max acceleration (m/s²)
    Units.degreesToRadians(540),  // Max angular velocity
    Units.degreesToRadians(460)   // Max angular acceleration
);
```

2. Configure the holonomic drive controller:
```java
public static final PPHolonomicDriveController PP_HOLONOMIC_DRIVE_CONTROLLER = 
    new PPHolonomicDriveController(
        new PIDConstants(2.4, 0, 0.015),  // Translation PID
        new PIDConstants(7.8, 0, 0.015)   // Rotation PID
    );
```

### Performance Tuning

In SwerveDriveSubsystemConstants.java:

1. Configure performance modes:
```java
public static final class PerformanceModeDefault {
    public static final double DRIVE_TRAIN_MAX_SPD = 3.5; // m/s
    public static final double DRIVE_TRAIN_MAX_ACCELERATION = 2.0; // m/s²
    public static final double DRIVE_TRAIN_MAX_ROT_SPD = 0.75 * 2 * Math.PI; // rad/s
}
```

2. Configure acceleration limits:
```java
public static final double DRIVE_XY_SPD_PERF_MODE_SW_FILTER_RATE = 8.0; // m/s/s
public static final double DRIVE_ROT_SPD_PERF_MODE_SW_FILTER_RATE = 4.0; // rad/s/s
```

## Telemetry Configuration

In SwerveDriveTelemetry.java:

1. Configure Shuffleboard layout:
```java
private ShuffleboardLayout initializePoseList(SwerveDriveSubsystem swerveDrive) {
    return driveTab
        .getLayout("Pose", BuiltInLayouts.kList)
        .withSize(2, 3)
        .withPosition(0, 0);
}
```

2. Add custom telemetry data:
```java
private void initializeOtherWidgets(SwerveDriveSubsystem swerveDrive) {
    driveTab.addBoolean("Custom Metric", () -> /* your condition */);
}
```

## Usage Examples

### Basic Drive Configuration
```java
// Create drivetrain using the appropriate year's TunerConstants
SwerveDriveSubsystem swerve = TunerConstants.createDrivetrain();

// Configure PathPlanner
SwerveDrivePathPlanner pathPlanner = new SwerveDrivePathPlanner(swerve);
```

### Autonomous Path Following
```java
// Load and run an autonomous path
Command autoPath = pathPlanner.getAutoPath("YourPathName");
autoPath.schedule();

// Create a pathfinding command to a specific pose
Command pathfindCommand = pathPlanner.getPathFinderCommand(
    new Pose2d(1, 1, new Rotation2d()),
    MetersPerSecond.of(0)
);
```

### Manual Drive Control
```java
swerve.applyRequest(() -> 
    new SwerveRequest.FieldCentric()
        .withVelocityX(joystickX)
        .withVelocityY(joystickY)
        .withRotation(joystickRotation)
);
```

## Best Practices

1. Module Configuration
    - Calibrate encoder offsets with the robot elevated and wheels pointing forward
    - Verify motor and encoder IDs match physical hardware
    - Double-check gear ratios against physical hardware

2. PathPlanner Usage
    - Keep path constraints within physical capabilities
    - Test autonomous paths at reduced speeds first
    - Use the PathPlanner GUI to visualize and verify paths

3. Performance Tuning
    - Start with conservative speed limits
    - Gradually increase limits while monitoring stability
    - Test all performance modes thoroughly

4. Telemetry
    - Monitor module states during testing
    - Use field visualization to verify odometry
    - Log relevant data for troubleshooting