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

  private final java.util.Map<DetectedObject, edu.wpi.first.wpilibj.Timer> detectionTimers =
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

            // If no match found in existing objects, check timers
            if (!matched) {
              boolean timerFound = false;
              edu.wpi.first.wpilibj.Timer matchingTimer = null;

              // Look for an existing timer for a similar position
              for (var entry : detectionTimers.entrySet()) {
                if (entry.getKey().matchesPosition(object)) {
                  timerFound = true;
                  matchingTimer = entry.getValue();
                  System.out.println("Found existing timer for similar position: " + object);
                  break;
                }
              }

              // If no timer found for this position, create new one
              if (!timerFound) {
                matchingTimer = new edu.wpi.first.wpilibj.Timer();
                matchingTimer.start();
                detectionTimers.put(object, matchingTimer);
                System.out.println("New timer created for position: " + object);
              }

              // Check if timer has elapsed
              if (matchingTimer.hasElapsed(
                  PhotonVisionObjectDetectionSubsystemConstants.MINIMUM_DETECTION_TIME)) {
                detectedObjects.add(object);
                System.out.println(
                    "Added new object after " + matchingTimer.get() + "s: " + object);
              } else {
                System.out.println("Waiting on timer: " + matchingTimer.get() + "s for: " + object);
              }
            }
          }
        }
      }
    }

    // Clean up old timers that haven't seen matches for the grace period
    int beforeSize = detectionTimers.size();
    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    detectionTimers
        .entrySet()
        .removeIf(
            entry -> {
              DetectedObject timerObject = entry.getKey();
              edu.wpi.first.wpilibj.Timer timer = entry.getValue();

              // Check if we've seen this object recently across any camera
              boolean seenRecently = false;
              for (PhotonCamera camera : cameras) {
                if (camera != null) {
                  var results = camera.getAllUnreadResults();
                  for (var result : results) {
                    if (result.hasTargets()) {
                      for (var target : result.getTargets()) {
                        DetectedObject currentObject =
                            DetectedObject.fromPhotonTarget(
                                target,
                                PhotonVisionObjectDetectionSubsystemConstants.CAMERA_OFFSETS[0],
                                driveSubsystem);

                        if (timerObject.matchesPosition(currentObject)) {
                          seenRecently = true;
                          timer.reset(); // Reset grace period timer when object is seen
                          System.out.println(
                              "Reset grace period for timer at position: " + timerObject);
                          break;
                        }
                      }
                    }
                    if (seenRecently) break;
                  }
                }
                if (seenRecently) break;
              }

              // Remove if we haven't seen it for the grace period
              boolean shouldRemove =
                  timer.get()
                      > PhotonVisionObjectDetectionSubsystemConstants.TIMER_CLEANUP_GRACE_PERIOD;

              if (shouldRemove) {
                System.out.println(
                    "Removing timer for position "
                        + timerObject
                        + " after "
                        + timer.get()
                        + "s without detection");
              }

              return shouldRemove;
            });

    if (beforeSize != detectionTimers.size()) {
      System.out.println(
          "Position timers changed from " + beforeSize + " to " + detectionTimers.size());
    }

    telemetry.updateObjects(detectedObjects);
  }
}
