package frc.alotobots.library.vision.photonvision.objectdetection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDriveSubsystem;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * A subsystem that manages multiple PhotonVision cameras for object detection. Handles game piece
 * and robot detection using computer vision.
 */
public class PhotonVisionObjectDetectionSubsystem extends SubsystemBase {
  private final PhotonCamera[] cameras;
  private final PhotonVisionObjectDetectionTelemetry telemetry;

  public PhotonVisionObjectDetectionSubsystem(SwerveDriveSubsystem driveSubsystem) {
    this.cameras = PhotonVisionObjectDetectionSubsystemConstants.CAMERAS;
    this.telemetry = new PhotonVisionObjectDetectionTelemetry();
    DetectedObject.setDrive(driveSubsystem);
    System.out.println("PhotonVisionObjectDetection Subsystem Initialized");
  }

  private final List<DetectedObject> detectedObjects = new ArrayList<>();

  /**
   * Gets the list of currently detected objects from enabled cameras only.
   *
   * @return List of DetectedObject instances from enabled cameras
   */
  public List<DetectedObject> getDetectedObjects() {
    return new ArrayList<>(detectedObjects);
  }

  @Override
  public void periodic() {
    detectedObjects.clear();
    System.out.println("PhotonVision periodic update starting...");

    // Process each camera
    for (int i = 0; i < cameras.length; i++) {
      // Skip this camera if it's disabled in telemetry
      if (!telemetry.isCameraEnabled(i)) {
        continue;
      }

      PhotonCamera camera = cameras[i];
      if (camera != null) {
        var result = camera.getLatestResult();

        if (result.hasTargets()) {
          // Get the best target (closest/largest)
          PhotonTrackedTarget target = result.getBestTarget();
          // Create DetectedObject using camera transform
          DetectedObject object =
              DetectedObject.fromPhotonTarget(
                  target, PhotonVisionObjectDetectionSubsystemConstants.CAMERA_OFFSETS[i]);

          detectedObjects.add(object);
        }
      }
    }

    // Update telemetry with latest data
    telemetry.updateObjects(detectedObjects);
  }
}
