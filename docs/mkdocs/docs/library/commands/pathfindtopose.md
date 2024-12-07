# PathfindToPose

The PathfindToPose functionality is built into PathPlanner's native pathfinding system, eliminating the need for a custom command implementation.

## Usage

```java
// Create a command that will pathfind to the specified pose
Command pathfindCommand = pathPlanner.getPathFinderCommand(
    targetPose,
    LinearVelocity.ofBaseUnits(0, Units.MetersPerSecond)
);
```

## Parameters

- `targetPose`: The target pose to pathfind to
- `LinearVelocity`: The constraints for the path, specified in meters per second

## Example

```java
// Example usage in a command
public Command getPathfindCommand(Pose2d targetPose) {
    return pathPlanner.getPathFinderCommand(
        targetPose,
        LinearVelocity.ofBaseUnits(4, Units.MetersPerSecond)  // 4 meters per second
    );
}
```

## Notes

- The command will automatically handle pathfinding to avoid obstacles
- Uses PathPlanner's built-in pathfinding algorithms
- No additional configuration needed beyond standard PathPlanner setup
