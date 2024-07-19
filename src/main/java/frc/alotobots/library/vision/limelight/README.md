# Limelight Subsystem

## Overview
The Limelight subsystem is responsible for interfacing with the Limelight vision system, primarily used for object detection and tracking.

## Files
- `LimelightSubsystem.java`: Main subsystem class that interfaces with the Limelight vision system and manages detected objects.
- `LimelightSubsystemConstants.java`: Contains constants used by the Limelight subsystem, such as camera offset and network table names.
- `LimelightTelemetry.java`: Handles telemetry for the Limelight subsystem, including Shuffleboard integration.
- `LimelightLib.java`: A utility library for interacting with the Limelight camera.
- `DetectedObject.java`: Represents an object detected by the Limelight system.
- `DetectedObjectList.java`: Manages a list of detected objects, including sorting and filtering capabilities.

## Dependencies
- Bling Subsystem: Used for visual feedback (optional).
- SwerveDrive Subsystem: Used for pose-related calculations in object detection.

## Dependent Subsystems
No other subsystems appear to be directly dependent on the Limelight subsystem based on the provided code.

## Commands
No specific commands are directly associated with this subsystem based on the provided code. It primarily provides object detection and tracking services that can be used by other subsystems and commands.

## Summary
The Limelight subsystem provides an interface to the Limelight vision system, allowing the robot to detect and track objects in its environment. It maintains a list of detected objects, supports different object types (e.g., game pieces, robots), and provides methods for calculating distances and angles to detected objects. The subsystem includes telemetry features for monitoring the Limelight's status and detected objects. It can work in conjunction with the robot's drive system to provide pose-relative object information.
