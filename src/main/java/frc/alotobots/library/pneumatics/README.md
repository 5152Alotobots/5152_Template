# Pneumatics Subsystem

## Overview
The Pneumatics subsystem is responsible for controlling the pneumatic system on the robot, including the compressor and potentially pneumatic actuators.

## Files
- `PneumaticsSubsystem.java`: Main subsystem class that controls the pneumatic system, primarily focusing on compressor management.
- `PneumaticsSubsystemConstants.java`: Contains constants used by the Pneumatics subsystem (currently empty in the provided code).
- `PneumaticsTelemetry.java`: Handles telemetry for the Pneumatics subsystem, including Shuffleboard integration.

## Dependencies
This subsystem does not appear to have direct dependencies on other subsystems within the project.

## Dependent Subsystems
No other subsystems appear to be directly dependent on the Pneumatics subsystem based on the provided code.

## Commands
No specific commands are directly associated with this subsystem based on the provided code. However, the subsystem provides methods that could be used in commands for controlling the compressor.

## Summary
The Pneumatics subsystem manages the robot's pneumatic system, primarily focusing on compressor control. It provides methods to enable and disable the compressor, check its running state, monitor current draw, and read the pressure switch. The subsystem includes telemetry features for monitoring the pneumatic system's status through Shuffleboard. While the current implementation focuses on compressor management, the subsystem could be expanded to control other pneumatic components such as solenoids or cylinders as needed for robot mechanisms.
