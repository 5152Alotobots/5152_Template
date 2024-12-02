package frc.alotobots.library.vision.photonvision.objectdetection;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDriveSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import lombok.Getter;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * A subsystem that manages multiple PhotonVision cameras for object detection. Handles game piece
 * and robot detection using computer vision.
 */
public class PhotonVisionObjectDetectionSubsystem extends SubsystemBase {
  private final PhotonCamera[] cameras;
  private final PhotonVisionObjectDetectionTelemetry telemetry;
  private final SwerveDriveSubsystem driveSubsystem;

  public PhotonVisionObjectDetectionSubsystem(SwerveDriveSubsystem driveSubsystem) {
    this.cameras = PhotonVisionObjectDetectionSubsystemConstants.CAMERAS;
    this.telemetry = new PhotonVisionObjectDetectionTelemetry();
    this.driveSubsystem = driveSubsystem;
    System.out.println("PhotonVisionObjectDetection Subsystem Initialized");
  }

  /**
   * -- GETTER -- Gets the list of currently detected objects from enabled cameras only.
   *
   * @return List of DetectedObject instances from enabled cameras
   */
  @Getter private final List<DetectedObject> detectedObjects = new ArrayList<>();

  // Smoothing factor for exponential smoothing (0 to 1)
  private static final double SMOOTHING_FACTOR = 0.05;

  // Last smoothed angle value
  private double lastSmoothedAngle = 0.0;
  private boolean hasValidMeasurement = false;

  /**
   * Gets the exponentially smoothed yaw angle to the first detected object.
   *
   * @return Optional containing the smoothed angle in radians, or empty if no objects are detected
   */
  public Optional<Double> getFieldRelativeAngle() {
    if (detectedObjects.isEmpty()) {
      hasValidMeasurement = false;
      return Optional.empty();
    }

    // Calculate raw angle in radians
    double robotAngle = driveSubsystem.getState().Pose.getRotation().getDegrees();
    double rawAngle =
        Units.degreesToRadians(robotAngle + detectedObjects.get(0).getTarget().getYaw());

    // Apply exponential smoothing
    if (!hasValidMeasurement) {
      // First measurement, initialize smoothing
      lastSmoothedAngle = rawAngle;
      hasValidMeasurement = true;
    } else {
      // Apply smoothing formula: smoothed = α * current + (1 - α) * lastSmoothed
      lastSmoothedAngle = SMOOTHING_FACTOR * rawAngle + (1 - SMOOTHING_FACTOR) * lastSmoothedAngle;
    }

    return Optional.of(lastSmoothedAngle);
  }

  @Override
  public void periodic() {
    // Check if object detection should be running based on dashboard toggle
    if (!telemetry.isObjectDetectionEnabled()) {
      detectedObjects.clear();
      telemetry.updateObjects(detectedObjects);
      return;
    }

    // Check if we should only run in teleop based on dashboard toggle
    if (telemetry.isTeleopOnlyEnabled() && !DriverStation.isTeleopEnabled()) {
      detectedObjects.clear();
      telemetry.updateObjects(detectedObjects);
      return;
    }

    detectedObjects.clear();

    // Process each camera
    for (int i = 0; i < cameras.length; i++) {
      // Skip this camera if it's disabled in telemetry
      if (!telemetry.isCameraEnabled(i)) {
        continue;
      }

      PhotonCamera camera = cameras[i];
      if (camera != null) {
        var result = camera.getAllUnreadResults();

        if (!result.isEmpty()) {
          // Get all targets from this camera
          for (PhotonTrackedTarget target : result.get(0).getTargets()) {
            // Create DetectedObject using camera transform for each target
            DetectedObject object =
                DetectedObject.fromPhotonTarget(
                    target,
                    PhotonVisionObjectDetectionSubsystemConstants.CAMERA_OFFSETS[i],
                    driveSubsystem);

            detectedObjects.add(object);
          }
        }
      }
    }

    // Update telemetry with latest data
    telemetry.updateObjects(detectedObjects);
  }
}
