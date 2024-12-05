# Team 5152 Robot Code Documentation

Welcome to the comprehensive documentation for Team 5152's robot code. This documentation provides an in-depth look at our robot's sophisticated control and automation systems.

## Core Systems

Our robot's drive system is built around an advanced [SwerveDrive Subsystem](./library/subsystems/swerve.md) that provides precise omnidirectional control. This integrates seamlessly with our [Controls System](./game/subsystems/controls.md), which handles driver input processing and sophisticated deadband management. The drive system also incorporates autonomous path following and trajectory generation capabilities for consistent, repeatable performance.

Vision processing forms another cornerstone of our robot's capabilities. The [AprilTag System](./library/subsystems/vision/apriltag.md) provides field-relative pose estimation, while our [Object Detection](./library/subsystems/vision/objectdetection.md) system enables real-time game piece tracking. These systems work together through multi-camera fusion and filtering to maintain accurate spatial awareness during matches.

## Game-Specific Features

We've developed several specialized commands to enhance game performance. The [DriveFacingBestObject](./library/commands/drivefacingbestobject.md) command provides assisted game piece targeting, while [PathfindToBestObject](./library/commands/pathfindtobestobject.md) enables autonomous navigation to game pieces. Our [DefaultSetToAllianceColor](./library/commands/defaultsettoalliancecolor.md) command manages alliance-specific indicators.

## Support Infrastructure

Supporting these primary systems, our robot includes comprehensive [Pneumatics Control](./library/subsystems/pneumatics.md) for actuator and pressure management, along with an [LED Notification System](./library/subsystems/bling.md) that provides clear visual feedback about robot state and debugging information.

## Developer Resources

For detailed technical information, developers can access our [JavaDoc API Documentation](./javadoc/) and review our source code directly on [GitHub](https://github.com/5152Alotobots/5152_Template). Our codebase emphasizes modular design through a command-based architecture, making it easy to extend and maintain. The system includes comprehensive telemetry and debugging capabilities, ensuring smooth development and testing processes.
