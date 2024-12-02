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

  // Smoothing and control parameters
  private static final double SMOOTHING_FACTOR = 0.2; // Reduced for smoother response
  private static final int MOVING_AVERAGE_WINDOW = 5;
  private static final double DEADBAND_RADIANS = Math.toRadians(1.0); // 1 degree deadband
  private static final double APPROACH_FACTOR = 0.5; // Reduces aggressive corrections near target

  // Smoothing state variables
  private double lastSmoothedAngle = 0.0;
  private boolean hasValidMeasurement = false;
  private final double[] angleBuffer = new double[MOVING_AVERAGE_WINDOW];
  private int bufferIndex = 0;
  private int validSamples = 0;

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

    // First apply exponential smoothing
    if (!hasValidMeasurement) {
      lastSmoothedAngle = rawAngle;
      hasValidMeasurement = true;
    } else {
      lastSmoothedAngle = SMOOTHING_FACTOR * rawAngle + (1 - SMOOTHING_FACTOR) * lastSmoothedAngle;
    }

    // Update moving average buffer
    angleBuffer[bufferIndex] = lastSmoothedAngle;
    bufferIndex = (bufferIndex + 1) % MOVING_AVERAGE_WINDOW;
    validSamples = Math.min(validSamples + 1, MOVING_AVERAGE_WINDOW);

    // Calculate moving average
    double sum = 0;
    for (int i = 0; i < validSamples; i++) {
      sum += angleBuffer[i];
    }
    double movingAverage = sum / validSamples;

    // Apply deadband and approach factor
    if (Math.abs(movingAverage) < DEADBAND_RADIANS) {
      return Optional.of(0.0); // Within deadband, return zero to stop movement
    } else {
      // Reduce the magnitude of the angle as we get closer to the target
      double scaleFactor = Math.min(1.0, 
          (Math.abs(movingAverage) - DEADBAND_RADIANS) / Math.toRadians(10.0) + APPROACH_FACTOR);
      return Optional.of(movingAverage * scaleFactor);
    }
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
