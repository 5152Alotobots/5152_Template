//   ____  _ ____ ____       _    _        ___ _____ ___  ____     ___ _____ ____
//  | ___|/ | ___|___ \    / \  | |     / _ \_   _/ _ \| __) / _ \_   _/ ___|
//  |___ \| |___ \ __) | / _ \ | |    | | | || || | | |  _ \|| | || | \___ \
//   ___) | |___) / __/   / ___ \| |__  | |_| || || |_| | |_) | |_| || |  ___) |
//  |____/|_|____/_____| /_/   \_\_____\___/ |_| \___/|____/ \___/ |_| |____/
//
//
// 2025 ALOTOBOTS FRC 5152
// Robot Code
package frc.alotobots.library.subsystems.vision.photonvision.objectdetection;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.constants.ObjectDetectionConstants;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io.ObjectDetectionIO;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io.ObjectDetectionInputsAutoLogged;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.util.DetectedObject;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class ObjectDetectionSubsystem extends SubsystemBase {
  private final ObjectDetectionIO[] io;
  private final ObjectDetectionInputsAutoLogged[] inputs;
  private final SwerveDriveSubsystem driveSubsystem;
  private List<DetectedObject> detectedObjects = new ArrayList<>();
  private final HashMap<DetectedObject, Timer> detectionTimers = new HashMap<>();

  // Lists for per-camera logging
  private final List<List<Pose3d>> confirmedPosesPerCamera;
  private final List<List<Pose3d>> pendingPosesPerCamera;

  public ObjectDetectionSubsystem(SwerveDriveSubsystem driveSubsystem, ObjectDetectionIO... io) {
    this.io = io;
    this.inputs = new ObjectDetectionInputsAutoLogged[io.length];
    this.confirmedPosesPerCamera = new ArrayList<>(io.length);
    this.pendingPosesPerCamera = new ArrayList<>(io.length);

    for (int i = 0; i < io.length; i++) {
      inputs[i] = new ObjectDetectionInputsAutoLogged();
      confirmedPosesPerCamera.add(new ArrayList<>());
      pendingPosesPerCamera.add(new ArrayList<>());
    }
    this.driveSubsystem = driveSubsystem;
  }

  @Override
  public void periodic() {
    // Clear per-camera logging lists
    for (int i = 0; i < io.length; i++) {
      confirmedPosesPerCamera.get(i).clear();
      pendingPosesPerCamera.get(i).clear();
    }

    // Update all camera inputs
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
    }

    // Update confidence and remove stale objects
    List<DetectedObject> updatedObjects = new ArrayList<>();
    for (DetectedObject obj : detectedObjects) {
      DetectedObject updated = obj.withUpdatedConfidence();
      if (!updated.isStale()) {
        updatedObjects.add(updated);
        // Add to confirmed poses for the appropriate camera
        for (int i = 0; i < io.length; i++) {
          if (inputs[i].cameraToRobot.equals(updated.robotToCamera())) {
            confirmedPosesPerCamera.get(i).add(updated.pose());
            break;
          }
        }
      }
    }
    detectedObjects = updatedObjects;

    // Process each camera's data
    for (int i = 0; i < inputs.length; i++) {
      if (!inputs[i].hasTargets) continue;

      // Process each target for this camera
      for (int t = 0; t < inputs[i].targetYaws.length; t++) {
        DetectedObject object =
            DetectedObject.fromVisionData(
                inputs[i].targetYaws[t],
                inputs[i].targetPitches[t],
                inputs[i].targetClassIds[t],
                inputs[i].cameraToRobot,
                driveSubsystem);

        processDetectedObject(object);
      }
    }

    // Get pending objects from timers and sort by camera
    for (var entry : detectionTimers.entrySet()) {
      DetectedObject obj = entry.getKey();
      for (int i = 0; i < io.length; i++) {
        if (inputs[i].cameraToRobot.equals(obj.robotToCamera())) {
          pendingPosesPerCamera.get(i).add(obj.pose());
          break;
        }
      }
    }

    // Log per-camera data
    for (int i = 0; i < io.length; i++) {
      String cameraName = ObjectDetectionConstants.CAMERA_CONFIGS[i].name();
      Logger.recordOutput(
          "Vision/Camera" + cameraName + "/ConfirmedPoses",
          confirmedPosesPerCamera.get(i).toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraName + "/PendingPoses",
          pendingPosesPerCamera.get(i).toArray(new Pose3d[0]));
    }

    cleanupTimers();
  }

  private void processDetectedObject(DetectedObject object) {
    boolean matched = false;
    for (int i = 0; i < detectedObjects.size(); i++) {
      if (detectedObjects.get(i).matchesPosition(object)) {
        detectedObjects.set(i, detectedObjects.get(i).refresh());
        matched = true;
        break;
      }
    }

    if (!matched) {
      Timer matchingTimer = null;
      for (var entry : detectionTimers.entrySet()) {
        if (entry.getKey().matchesPosition(object)) {
          matchingTimer = entry.getValue();
          break;
        }
      }

      if (matchingTimer == null) {
        matchingTimer = new Timer();
        matchingTimer.start();
        detectionTimers.put(object, matchingTimer);
      }

      if (matchingTimer.hasElapsed(ObjectDetectionConstants.MINIMUM_DETECTION_TIME)) {
        detectedObjects.add(object);
      }
    }
  }

  private void cleanupTimers() {
    detectionTimers
        .entrySet()
        .removeIf(
            entry -> {
              boolean exceededGracePeriod =
                  entry.getValue().hasElapsed(ObjectDetectionConstants.TIMER_CLEANUP_GRACE_PERIOD);
              boolean isActivelyDetected =
                  detectedObjects.stream().anyMatch(obj -> obj.matchesPosition(entry.getKey()));
              return exceededGracePeriod && !isActivelyDetected;
            });
  }

  public List<DetectedObject> getDetectedObjects() {
    return new ArrayList<>(detectedObjects);
  }
}
