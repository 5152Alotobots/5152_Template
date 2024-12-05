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

  private final java.util.Map<PhotonTrackedTarget, edu.wpi.first.wpilibj.Timer> detectionTimers =
      new java.util.HashMap<>();

  /**
   * Gets the raw field-relative angle to the first detected object.
   *
   * @return Optional containing the angle in radians, or empty if no objects are detected
   */
  public Optional<Double> getFieldRelativeAngle() {
    if (detectedObjects.isEmpty()) {
      return Optional.empty();
    }

    double robotAngle = driveSubsystem.getState().Pose.getRotation().getDegrees();
    return Optional.of(
        Units.degreesToRadians(robotAngle + detectedObjects.get(0).getTarget().getYaw()));
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

    // Update confidence for existing objects and remove stale ones
    detectedObjects.removeIf(
        obj -> {
          obj.updateConfidence();
          return obj.isStale();
        });

    // Process each camera
    for (int i = 0; i < cameras.length; i++) {
      // Skip this camera if it's disabled in telemetry
      if (!telemetry.isCameraEnabled(i)) {
        continue;
      }

      PhotonCamera camera = cameras[i];
      if (camera != null) {
        var results = camera.getAllUnreadResults();

        for (var result : results) {
          if (!result.hasTargets()) continue;

          // Get all targets from this camera
          for (PhotonTrackedTarget target : result.getTargets()) {
            // Create DetectedObject using camera transform for each target
            DetectedObject object =
                DetectedObject.fromPhotonTarget(
                    target,
                    PhotonVisionObjectDetectionSubsystemConstants.CAMERA_OFFSETS[i],
                    driveSubsystem);

            // Check if this object matches any existing ones
            boolean matched = false;
            for (DetectedObject existing : detectedObjects) {
              if (existing.matchesPosition(object)) {
                // Update existing object
                existing.refresh();
                matched = true;
                break;
              }
            }

            // If no match found, check/update timer for this target
            if (!matched) {
              edu.wpi.first.wpilibj.Timer timer =
                  detectionTimers.computeIfAbsent(
                      target,
                      k -> {
                        var t = new edu.wpi.first.wpilibj.Timer();
                        t.start();
                        System.out.println(
                            "New timer created for target: " + target.getFiducialId());
                        return t;
                      });

              if (timer.hasElapsed(
                  PhotonVisionObjectDetectionSubsystemConstants.MINIMUM_DETECTION_TIME)) {
                detectedObjects.add(object);
                System.out.println("Added new object after " + timer.get() + "s: " + object);
              } else {
                System.out.println(
                    "Waiting on timer: " + timer.get() + "s for target: " + target.getFiducialId());
              }
            }
          }
        }
      }
    }

    // Clean up old timers that don't match any current targets
    int beforeSize = detectionTimers.size();
    detectionTimers
        .entrySet()
        .removeIf(
            entry -> {
              boolean shouldRemove = true;
              for (PhotonCamera camera : cameras) {
                if (camera != null) {
                  var results = camera.getAllUnreadResults();
                  for (var result : results) {
                    if (result.hasTargets()) {
                      System.out.println(
                          "Checking " + result.getTargets().size() + " targets from camera");
                      for (var target : result.getTargets()) {
                        if (target.equals(entry.getKey())) {
                          shouldRemove = false;
                          System.out.println("Keeping timer for target: " + target.getFiducialId());
                          break;
                        }
                      }
                    }
                  }
                }
              }
              if (shouldRemove) {
                System.out.println("Removing timer for target: " + entry.getKey().getFiducialId());
              }
              return shouldRemove;
            });

    if (beforeSize != detectionTimers.size()) {
      System.out.println("Timers changed from " + beforeSize + " to " + detectionTimers.size());
    }

    telemetry.updateObjects(detectedObjects);
  }
}
