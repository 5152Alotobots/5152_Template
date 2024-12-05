# PhotonVision Object Detection Subsystem
## Overview
The PhotonVision Object Detection System is a sophisticated computer vision subsystem designed for FRC robots. It provides real-time detection and tracking of game pieces and other objects using multiple PhotonVision cameras. The system maintains object persistence across frames, handles confidence decay, and provides comprehensive telemetry data for debugging and visualization.

## Core Components

### DetectedObject Class
The DetectedObject class serves as the fundamental unit of object tracking. It encapsulates:
- 3D pose information in field coordinates
- Raw PhotonVision target data
- Camera-to-robot transform data
- Confidence tracking
- Distance and angle calculations

Each detected object maintains a confidence value that decays over time when the object is not actively detected, helping filter out spurious detections while maintaining object persistence when detection is briefly lost.

### PhotonVisionObjectDetectionSubsystem
This is the main subsystem class that orchestrates the entire detection system. Key responsibilities include:
1. Managing multiple PhotonVision cameras
2. Processing incoming vision data
3. Maintaining the list of detected objects
4. Handling object persistence and matching
5. Managing detection timers for filtering false positives

The subsystem implements several sophisticated mechanisms:
- **Object Matching**: Uses position-based matching to update existing objects rather than creating duplicates
- **Confidence Decay**: Gradually reduces confidence in objects that haven't been seen recently
- **Detection Timers**: Requires consistent detection over time before adding new objects
- **Multi-Camera Integration**: Processes and combines data from multiple cameras

### Telemetry System
The telemetry component provides comprehensive debugging and visualization capabilities:
- Real-time field visualization
- Per-camera statistics and controls
- Object tracking visualization with tracer lines
- Confidence and position data for detected objects
- Toggle switches for enabling/disabling features

## System Operation

### Detection Pipeline
1. **Camera Processing**
    - Each enabled camera's latest frame is processed
    - Raw targets are extracted from PhotonVision
    - Targets are converted to field-relative 3D poses

2. **Object Management**
    - New detections are matched against existing objects
    - Matched objects have their confidence refreshed
    - Unmatched detections start a detection timer
    - Objects passing the minimum detection time are added to tracking

3. **Maintenance**
    - Confidence values decay over time
    - Stale objects are removed
    - Detection timers are cleaned up
    - Telemetry is updated

### Object Persistence
The system uses several mechanisms to maintain reliable object tracking:
- Position-based matching within a configurable tolerance
- Confidence decay for graceful degradation
- Detection timers to prevent false positives
- Grace periods for temporary detection loss

## Configuration and Tuning

### Constants Configuration
Key parameters that need tuning in PhotonVisionObjectDetectionSubsystemConstants:

1. **Camera Configuration**
    - `CAMERAS`: Array of PhotonCamera objects
    - `CAMERA_OFFSETS`: 3D transforms from robot center to each camera
   ```java
   new Transform3d(
       new Translation3d(x, y, z),
       new Rotation3d(roll, pitch, yaw)
   )
   ```

2. **Detection Parameters**
    - `INITIAL_CONFIDENCE` (0.99): Starting confidence for new detections
    - `CONFIDENCE_DECAY_RATE` (0.33): How quickly confidence decreases
    - `MIN_CONFIDENCE` (0.0): Threshold for removing objects
    - `POSITION_MATCH_TOLERANCE` (0.5m): Distance threshold for matching objects
    - `MINIMUM_DETECTION_TIME` (0.25s): Required consistent detection time
    - `TIMER_CLEANUP_GRACE_PERIOD` (5.0s): How long to maintain detection timers

### Adding New Cameras
To add a new camera:
1. Add a new PhotonCamera instance to the `CAMERAS` array:
   ```java
   public static final PhotonCamera[] CAMERAS = new PhotonCamera[] {
       new PhotonCamera("Camera1"),
       new PhotonCamera("Camera2")  // New camera
   };
   ```
2. Add corresponding offset transform in `CAMERA_OFFSETS`:
   ```java
   public static final Transform3d[] CAMERA_OFFSETS = new Transform3d[] {
       existing_transform,
       new Transform3d(
           new Translation3d(x, y, z),
           new Rotation3d(roll, pitch, yaw)
       )
   };
   ```

### Adding New Game Elements
To add new detectable objects:
1. Add new GameElement to `GAME_ELEMENTS` array:
   ```java
   public static final GameElement[] GAME_ELEMENTS = new GameElement[] {
       existing_element,
       new GameElement("NewObject", length, width, height)
   };
   ```
2. Ensure the array index matches the class ID from PhotonVision's model

## Best Practices

1. **Camera Placement**
    - Mount cameras with clear fields of view
    - Minimize exposure to bright lights and sun
    - Ensure rigid mounting to prevent vibration
    - Accurately measure and configure offsets

2. **Tuning**
    - Start with higher confidence decay rates
    - Adjust position tolerance based on field size
    - Tune minimum detection time based on frame rate
    - Set appropriate grace periods for your game piece dynamics

3. **Performance**
    - Enable only necessary cameras
    - Use teleop-only mode when appropriate
    - Monitor CPU usage and adjust parameters
    - Clean up stale objects and timers regularly