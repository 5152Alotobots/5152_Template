# SwerveDrive Subsystem

## Overview
The SwerveDrive subsystem is responsible for controlling the robot's swerve drive system. It provides functionality for robot movement, position tracking, and autonomous path following.

## Files
- `SwerveDriveSubsystem.java`: Main subsystem class that extends CTRE's SwerveDrivetrain and implements WPILib's Subsystem interface.
- `SwerveDriveTelemetry.java`: Handles telemetry for the SwerveDrive subsystem, including Shuffleboard integration.
- `SwerveDrivePathPlanner.java`: Configures and manages PathPlanner integration for autonomous routines.
- `AimModule.java`: Provides utility methods for aiming calculations.
- `TunerConstants.java`: Contains tuning constants for the swerve drive, including PID gains and gear ratios.

## Dependencies
- Photonvision Subsystem: Used for vision-based pose estimation.
- Bling Subsystem: May be used for visual feedback (not directly used in the provided code).

## Dependent Subsystems
No other subsystems appear to be directly dependent on the SwerveDrive subsystem based on the provided code.

## Commands
1. `DriveWhileFacingPose`: Allows the robot to drive while facing a specific pose.

## Summary
The SwerveDrive subsystem provides high-level control over the robot's movement using a swerve drive system. It integrates with vision systems for improved pose estimation, supports autonomous path following using PathPlanner, and provides telemetry data for debugging and tuning. The subsystem is designed to be flexible and configurable, with support for different swerve module configurations and tuning parameters.