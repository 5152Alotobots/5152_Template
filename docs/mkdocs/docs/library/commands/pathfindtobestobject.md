# PathfindToBestObject Command

## Purpose
The PathfindToBestObject command creates and executes a path for the robot to navigate to the closest detected game piece while maintaining a safe approach distance based on the robot's bumper dimensions. This instant command generates and schedules a pathfinding command that considers the robot's physical dimensions and optimal approach angles.

Related features:
- [DriveFacingBestObject](./drivefacingbestobject.md) for manual driving toward objects
- [Object Detection System](../subsystems/vision/objectdetection.md) for vision processing
- [SwerveDrive](../subsystems/swerve.md) for path following

## Required Subsystems
- [PhotonVision Object Detection System](../subsystems/vision/objectdetection.md)
- [SwerveDrive Subsystem](../subsystems/swerve.md)
- SwerveDrivePathPlanner

## Constructor
```java
public PathfindToBestObject(
    PhotonVisionObjectDetectionSubsystem objectDetectionSubsystem,
    SwerveDriveSubsystem swerveDriveSubsystem,
    SwerveDrivePathPlanner pathPlanner,
    String targetGameElementName
)
```

## Operation Details
The command performs several key calculations during initialization:

1. Filters detected objects to match the specified game element type
2. Determines the best matching object from the filtered results
2. Calculates the approach vector from robot to object
3. Selects appropriate bumper offset based on approach angle
4. Generates a target pose offset from the object for safe approach
5. Creates and schedules a PathPlanner command to the calculated position

## Smart Approach System
The command implements intelligent approach positioning by:
- Using the robot's actual bumper dimensions (length and width)
- Dynamically selecting the appropriate bumper dimension based on approach angle
- Calculating an offset position that prevents collision with the target object

## Usage Example
```java
new PathfindToBestObject(
    visionSubsystem,
    driveSubsystem,
    pathPlanner,
    "gamePiece"  // Specify the target game element name
).schedule();
```

## Important Notes
- The command executes instantly but schedules a longer-running path following command
- Returns immediately if no matching game elements are detected
- Only considers objects that match the specified game element type
- The target pose maintains the object's rotation
- Sets terminal velocity to 0 m/s for safe approach
- Requires accurate bumper dimensions in TunerConstants

The command is particularly useful for autonomous sequences and driver assistance features where precise positioning near game pieces is required.
