# Photonvision Subsystem

## Overview
The Photonvision subsystem is responsible for processing visual data from cameras on the robot, primarily for AprilTag detection and pose estimation.

## Files
- `PhotonvisionSubsystem.java`: Main subsystem class that manages multiple PhotonVision cameras and provides pose estimation functionality.
- `PhotonvisionSubsystemConstants.java`: Contains constants used by the Photonvision subsystem, such as camera configurations and pose estimation settings.
- `PhotonvisionTelemetry.java`: Handles telemetry for the Photonvision subsystem, including Shuffleboard integration.

## Dependencies
This subsystem does not appear to have direct dependencies on other subsystems within the project.

## Dependent Subsystems
- SwerveDrive Subsystem: Uses the Photonvision subsystem for vision-based pose estimation.

## Commands
No specific commands are directly associated with this subsystem based on the provided code. It primarily provides services to other subsystems and commands.

## Summary
The Photonvision subsystem integrates multiple cameras using the PhotonVision library to detect AprilTags and estimate the robot's pose on the field. It supports multi-camera setups and provides pose estimates that can be used by other subsystems (like the SwerveDrive subsystem) to improve localization accuracy. The subsystem also includes telemetry features for debugging and monitoring vision system performance.