# Team 5152 Robot Code Documentation

Welcome to the comprehensive documentation for Team 5152's robot code. This documentation provides detailed information about our robot's systems, subsystems, and commands.

## Core Systems

### Drive Systems
- [SwerveDrive Subsystem](./library/subsystems/swerve.md) - Advanced omnidirectional drive control
- [Controls System](./game/subsystems/controls.md) - Driver input processing with deadband handling
- Autonomous path following and trajectory generation

### Vision Processing
- [AprilTag System](./library/subsystems/vision/apriltag.md) - Field-relative pose estimation
- [Object Detection](./library/subsystems/vision/objectdetection.md) - Game piece tracking
- Multi-camera fusion and filtering

### Game-Specific Commands
- [DriveFacingBestObject](./library/commands/drivefacingbestobject.md) - Assisted game piece targeting
- [PathfindToBestObject](./library/commands/pathfindtobestobject.md) - Autonomous game piece approach
- [DefaultSetToAllianceColor](./library/commands/defaultsettoalliancecolor.md) - Alliance indicator control

### Support Systems
- [Pneumatics Control](./library/subsystems/pneumatics.md) - Actuator and pressure management
- [LED Notification](./library/subsystems/bling.md) - Visual state indication and debugging

## Developer Resources

### Quick Links
- [JavaDoc API Documentation](./javadoc/) - Detailed API reference
- [Source Code on GitHub](https://github.com/5152Alotobots/5152_Template)

### Key Features
- Sophisticated swerve drive control with field-relative operation
- Advanced vision processing with multi-camera support
- Comprehensive telemetry and debugging systems
- Modular command-based architecture
