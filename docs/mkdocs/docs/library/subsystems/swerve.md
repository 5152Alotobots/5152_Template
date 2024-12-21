# SwerveDrive Subsystem

## Overview
The SwerveDrive subsystem is a sophisticated drive system that enables omnidirectional movement of the robot. It manages four swerve modules (each containing drive and turn motors), handles odometry calculations, and provides autonomous path-following capabilities.

## Constructor Parameters
- `GyroIO gyroIO`: Interface for the gyroscope
- `ModuleIO flModuleIO`: Front left swerve module interface
- `ModuleIO frModuleIO`: Front right swerve module interface
- `ModuleIO blModuleIO`: Back left swerve module interface
- `ModuleIO brModuleIO`: Back right swerve module interface

## Commands that use the subsystem
- [DefaultDrive Command](../../library/commands/swerve/defaultdrive.md): Provides standard teleoperated control
- [DriveFacingAngle Command](../../library/commands/swerve/drivefacingangle.md): Maintains a specific robot heading while driving
- [DriveFacingPose Command](../../library/commands/swerve/drivefacingpose.md): Orients the robot to face a target position while driving
- [Characterization Commands](../../library/commands/swerve/characterization.md): Used for system identification and calibration

## Configuration
All configuration for the SwerveDrive subsystem is handled through tuner constants. This includes:
- Module configurations (gear ratios, conversions, PID values)
- PathPlanner and autonomous settings
- Drive characterization values
- Maximum speeds and acceleration limits

For more details on configuration, refer to the [TunerConstants documentation](../../game/tunerconstants.md).

## JavaDoc Reference
For detailed technical documentation, see the [SwerveDrive JavaDoc](/javadoc/frc/alotobots/library/subsystems/swervedrive/SwerveDriveSubsystem.html)