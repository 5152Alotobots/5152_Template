# Bling Subsystem

## Overview
The Bling subsystem is responsible for controlling LED lighting on the robot, providing visual feedback for various robot states and actions.

## Files
- `BlingSubsystem.java`: Main subsystem class that controls the LED lighting system using a CTRE CANdle controller.
- `BlingSubsystemConstants.java`: Contains constants used by the Bling subsystem, such as LED configurations and predefined colors and animations.
- `BlingTelemetry.java`: Handles telemetry for the Bling subsystem, including Shuffleboard integration.

## Dependencies
This subsystem does not appear to have direct dependencies on other subsystems within the project.

## Dependent Subsystems
- Limelight Subsystem: May use the Bling subsystem for visual feedback (optional).
- Other subsystems may potentially use the Bling subsystem for status indication, though not explicitly shown in the provided code.

## Commands
- `Cmd_SubSys_Bling_DefaultSetToAllianceColor`: Sets the LED color to match the current alliance color.

## Summary
The Bling subsystem provides control over the robot's LED lighting system. It supports setting solid colors, running animations, and queueing both colors and animations for sequential display. The subsystem can automatically set the LED color based on the alliance color reported by the Driver Station. It includes telemetry features for monitoring the current LED state and supports runtime enabling/disabling of the LED system. The Bling subsystem can be used by other subsystems and commands to provide visual feedback about the robot's state, actions, or detected objects.
