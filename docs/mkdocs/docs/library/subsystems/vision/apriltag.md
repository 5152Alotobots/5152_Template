# PhotonVision AprilTag Subsystem

## System Architecture

The PhotonVision AprilTag subsystem implements a robust vision-based pose estimation system using multiple cameras to track AprilTag fiducial markers. The system processes multiple camera feeds simultaneously, fuses their data, and produces highly accurate pose estimates through sophisticated filtering and smoothing algorithms.

## Core Components

The system is built around three main components that work together:

The PhotonvisionAprilTagSubsystem serves as the primary controller, managing the vision pipeline and pose estimation process. It interfaces with multiple PhotonCamera instances, each configured with specific mounting offsets and orientations relative to the robot's center. The subsystem maintains a collection of PhotonPoseEstimator objects that handle the geometric calculations for each camera.

The PhotonvisionAprilTagTelemetry component provides real-time visualization and debugging capabilities, managing the data flow to Shuffleboard for monitoring system performance and pose estimates.

PhotonvisionAprilTagSubsystemConstants contains the configuration parameters that define camera positions, filtering thresholds, and system behavior settings.

## System Configuration and Tuning

### Camera Addition and Configuration

Adding new cameras to the system requires modifications to both the hardware setup and software configuration. Each camera needs proper physical mounting with known offsets from the robot's center point, and corresponding configuration in the constants file.

To add a new camera:

1. Mount the camera securely on the robot
2. Measure the camera's position relative to robot center:
    - X: Distance forward from robot center (positive forward)
    - Y: Distance left from robot center (positive left)
    - Z: Height from ground (positive up)
3. Measure the camera's rotation:
    - Roll: Rotation around forward axis
    - Pitch: Rotation around side axis
    - Yaw: Rotation around vertical axis

4. Add the camera configuration to PhotonvisionAprilTagSubsystemConstants:

```java
// Add camera object to CAMERAS array
public static final PhotonCamera[] CAMERAS = new PhotonCamera[] {
    new PhotonCamera("FL_AprilTag"),
    new PhotonCamera("FM_AprilTag"),
    new PhotonCamera("NewCamera_AprilTag")  // Add new camera
};

// Add corresponding offset to CAMERA_OFFSETS array
public static final Transform3d[] CAMERA_OFFSETS = new Transform3d[] {
    // Existing cameras...
    new Transform3d(
        new Translation3d(x, y, z),
        new Rotation3d(roll, pitch, yaw)
    )
};
```

### Critical Tuning Parameters

The system's performance depends heavily on proper tuning of several key parameters in PhotonvisionAprilTagSubsystemConstants:

#### Vision Standard Deviations
```java
public static final Matrix<N3, N1> VISION_STD_DEVS
```
These values determine how much the system trusts vision measurements relative to odometry:
- Lower values (0.001-0.1): High trust in vision
- Higher values (0.5-1.0): Lower trust in vision
- Start with higher values and decrease gradually while monitoring stability

#### Pose Filter Parameters
```java
public static final double MAX_POSE_DEVIATION_METERS = 1.0;
public static final double MIN_TAG_WEIGHT = 0.3;
public static final double MAX_TAG_WEIGHT = 1.0;
```
These parameters control the outlier rejection and pose weighting system:
- MAX_POSE_DEVIATION_METERS: Increase if legitimate poses are being rejected, decrease if false poses are being accepted
- MIN/MAX_TAG_WEIGHT: Adjust the influence of single vs multi-tag detections

#### Smoothing Parameters
```java
public static final double POSITION_ALPHA = 0.05;
public static final double ROTATION_ALPHA = 0.05;
```
These control the trade-off between smoothness and responsiveness:
- Lower values (0.01-0.1): Smoother but more latent
- Higher values (0.1-0.3): More responsive but potentially jittery
- Position and rotation can be tuned independently based on robot behavior

### Tuning Process

1. Start with conservative values:
    - High vision standard deviations
    - High MAX_POSE_DEVIATION_METERS
    - Low smoothing alphas

2. Test basic functionality:
    - Verify all cameras are detecting tags
    - Check pose estimates against known positions
    - Monitor for obvious outliers

3. Iterative refinement:
    - Gradually lower vision standard deviations while monitoring pose stability
    - Adjust MAX_POSE_DEVIATION_METERS to balance outlier rejection
    - Fine-tune smoothing alphas based on robot movement characteristics

4. Validation:
    - Test under competition conditions
    - Verify performance during rapid movement
    - Check reliability with partially obscured tags

Monitor the telemetry output during tuning to observe the effect of parameter changes on system performance. The field visualization and pose graphs are particularly useful for identifying issues during the tuning process.

## Vision Pipeline

### Data Acquisition and Processing

The vision pipeline begins in the periodic loop where each camera captures frames independently. The system processes these frames asynchronously to identify AprilTags using PhotonVision's detection algorithms. For each detected tag, the system calculates a camera-relative pose using PNP (Perspective-n-Point) algorithms.

The raw camera data undergoes several processing stages:

1. Tag Detection: Each camera processes its image to identify AprilTags in its field of view.
2. Pose Calculation: The system calculates individual pose estimates based on the detected tags.
3. Multi-Tag Fusion: When multiple tags are visible, their poses are combined for improved accuracy.
4. Cross-Camera Fusion: Poses from different cameras are merged using weighted averaging.

### Pose Estimation Algorithm

The pose estimation process employs a sophisticated multi-stage algorithm:

First, the system collects pose estimates from all available cameras. Each estimate includes information about the tags used for the calculation and their detection confidence. The system then applies an outlier rejection algorithm that compares poses against the median position, filtering out estimates that deviate significantly from the consensus.

The remaining poses undergo a weighted averaging process. The weighting system considers several factors:
- Number of visible tags (more tags increase weight)
- Tag pose ambiguity (lower ambiguity increases weight)
- Historical reliability of the camera

The weighted averaging process uses exponential smoothing to reduce jitter while maintaining responsiveness. Position and rotation are smoothed independently with configurable smoothing factors.

### Outlier Rejection System

The outlier rejection system uses a statistical approach to identify and remove unreliable pose estimates. It calculates the median position of all estimates and measures the deviation of each estimate from this median. Estimates that exceed a configurable maximum deviation threshold are excluded from the final pose calculation.

The system also considers the geometric consistency of poses when multiple tags are visible. If the relative positions of detected tags don't match their known field positions within tolerance, the system can identify and reject inconsistent measurements.

### Pose Smoothing Implementation

The pose smoothing system uses exponential smoothing with separate smoothing factors for position and rotation. This dual-factor approach allows fine-tuning of the smoothing behavior independently for translational and rotational movements.

The smoothing algorithm maintains state between updates, storing the last smoothed pose. New pose estimates are combined with this historical data using the configurable alpha factors:

```
smoothed_value = alpha * new_value + (1 - alpha) * previous_smoothed_value
```

Lower alpha values produce smoother output but increase latency, while higher values provide more responsive updates but may introduce more jitter.

### Multi-Camera Fusion

The multi-camera fusion system combines pose estimates from different perspectives to improve accuracy and reliability. Each camera contributes to the final pose estimate based on its current view quality and historical reliability.

The fusion process handles several challenges:
- Temporal alignment of measurements from different cameras
- Resolution of conflicting pose estimates
- Handling of partial visibility conditions
- Compensation for camera calibration uncertainties

The system uses a weighted average approach that considers the number of tags visible to each camera and the geometric quality of the view (based on tag size and angle in the image).

### Integration with Robot Odometry

The subsystem integrates with the robot's odometry system through the SwerveDriveSubsystem interface. Vision measurements are added to the odometry system with appropriate uncertainty values, allowing the robot's state estimator to fuse vision data with other sensor inputs.

The vision standard deviations are configurable through a covariance matrix, enabling fine-tuning of how much the system trusts vision measurements relative to other sensors.

## Error Handling and Robustness

The system implements comprehensive error handling to maintain operation under various failure conditions:

- Camera disconnections are detected and handled gracefully
- Individual camera failures don't compromise the entire system
- Network communication issues are managed through timeout mechanisms
- Invalid pose estimates are filtered out before they can affect robot behavior

The system can continue operating with reduced accuracy when some cameras are unavailable, automatically adjusting its fusion algorithms to work with the remaining functional cameras.
