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

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.constants.ObjectDetectionConstants;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io.ObjectDetectionIO;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io.ObjectDetectionInputsAutoLogged;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.util.GameElement;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

public class ObjectDetectionSubsystem extends SubsystemBase {
  private final ObjectDetectionIO[] io;
  private final ObjectDetectionInputsAutoLogged[] inputs;
  private final Supplier<Pose2d> poseSupplier;
  private List<DetectedObject> activeObjects = new ArrayList<>();
  private final LinkedHashMap<DetectedObject, Timer> detectionTimers = new LinkedHashMap<>();

  // Arrays to store poses for each camera
  private final Pose3d[][] confirmedPoses;
  private final Pose3d[][] pendingPoses;
  private final int[] confirmedPoseCount;
  private final int[] pendingPoseCount;
  private static final int MAX_POSES_PER_CAMERA = 50;

  public record DetectedObject(
      Pose3d pose, int classId, double confidence, double lastUpdateTime) {}

  public ObjectDetectionSubsystem(Supplier<Pose2d> poseSupplier, ObjectDetectionIO... io) {
    this.io = io;
    this.inputs = new ObjectDetectionInputsAutoLogged[io.length];
    this.poseSupplier = poseSupplier;
    this.confirmedPoses = new Pose3d[io.length][MAX_POSES_PER_CAMERA];
    this.pendingPoses = new Pose3d[io.length][MAX_POSES_PER_CAMERA];
    this.confirmedPoseCount = new int[io.length];
    this.pendingPoseCount = new int[io.length];

    for (int i = 0; i < io.length; i++) {
      inputs[i] = new ObjectDetectionInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    double currentTimestamp = Timer.getTimestamp();
    
    // Reset pose counts
    for (int i = 0; i < io.length; i++) {
      confirmedPoseCount[i] = 0;
      pendingPoseCount[i] = 0;
    }

    // Update all camera inputs and process detections
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);

      if (inputs[i].hasTargets) {
        Transform3d robotToCamera =
            ((inputs[i].targetYaws.length > 0)
                ? ObjectDetectionConstants.CAMERA_CONFIGS[i].robotToCamera()
                : new Transform3d());

        for (int t = 0; t < inputs[i].targetYaws.length; t++) {
          DetectedObject object =
              calculateDetectedObject(
                  inputs[i].targetYaws[t],
                  inputs[i].targetPitches[t],
                  inputs[i].targetClassIds[t],
                  robotToCamera,
                  currentTimestamp);

          if (object != null) {
            processDetectedObject(object, i);
          }
        }
      }
    }

    // Update confidence and remove stale objects
    List<DetectedObject> updatedObjects = new ArrayList<>();
    for (DetectedObject obj : activeObjects) {
      DetectedObject updated = updateConfidence(obj, currentTimestamp);
      if (!isStale(updated)) {
        updatedObjects.add(updated);
      }
    }
    activeObjects = updatedObjects;

    // Log camera data
    for (int i = 0; i < io.length; i++) {
      String cameraName = ObjectDetectionConstants.CAMERA_CONFIGS[i].name();

      // Create arrays of the exact size needed
      Pose3d[] confirmedArray = new Pose3d[confirmedPoseCount[i]];
      Pose3d[] pendingArray = new Pose3d[pendingPoseCount[i]];

      System.arraycopy(confirmedPoses[i], 0, confirmedArray, 0, confirmedPoseCount[i]);
      System.arraycopy(pendingPoses[i], 0, pendingArray, 0, pendingPoseCount[i]);

      Logger.recordOutput("Vision/Camera" + cameraName + "/ConfirmedPoses", confirmedArray);
      Logger.recordOutput("Vision/Camera" + cameraName + "/PendingPoses", pendingArray);
    }

    cleanupTimers();
  }

  private DetectedObject calculateDetectedObject(
      double yaw, double pitch, int classId, Transform3d robotToCamera, double timestamp) {
    if (classId >= ObjectDetectionConstants.GAME_ELEMENTS.length) {
      return null;
    }

    GameElement matchedElement = ObjectDetectionConstants.GAME_ELEMENTS[classId];

    // Calculate distance using the matched element's height
    double targetToCameraDistance =
        PhotonUtils.calculateDistanceToTargetMeters(
            robotToCamera.getZ(),
            matchedElement.height(),
            robotToCamera.getRotation().getY(),
            Units.degreesToRadians(pitch));

    // Get the 2D translation in camera space
    Translation2d targetToCamera2d =
        PhotonUtils.estimateCameraToTargetTranslation(
            targetToCameraDistance, Rotation2d.fromDegrees(-yaw));

    // Convert to 3D transform
    Transform3d cameraToTarget =
        new Transform3d(
            new Translation3d(
                targetToCamera2d.getX(), targetToCamera2d.getY(), matchedElement.height()),
            new Rotation3d());

    // Combine transforms to get target in robot space
    Transform3d targetToRobot = robotToCamera.plus(cameraToTarget);

    // Convert to field space if we have drive data
    Pose3d targetPose;
    if (poseSupplier != null) {
      Pose3d robotPose =
          new Pose3d(
              poseSupplier.get().getX(),
              poseSupplier.get().getY(),
              0.0,
              new Rotation3d(0, 0, poseSupplier.get().getRotation().getRadians()));
      targetPose = robotPose.transformBy(targetToRobot);
    } else {
      targetPose = new Pose3d().transformBy(targetToRobot);
    }

    return new DetectedObject(
        targetPose, classId, ObjectDetectionConstants.INITIAL_CONFIDENCE, timestamp);
  }

  private void processDetectedObject(DetectedObject object, int cameraIndex) {
    boolean matched = false;
    for (DetectedObject existing : activeObjects) {
      if (matchesPosition(existing, object)) {
        matched = true;
        if (confirmedPoseCount[cameraIndex] < MAX_POSES_PER_CAMERA) {
          confirmedPoses[cameraIndex][confirmedPoseCount[cameraIndex]] = object.pose();
          confirmedPoseCount[cameraIndex]++;
        }
        break;
      }
    }

    if (!matched) {
      Timer matchingTimer = null;
      for (var entry : detectionTimers.entrySet()) {
        if (matchesPosition(entry.getKey(), object)) {
          matchingTimer = entry.getValue();
          break;
        }
      }

      if (matchingTimer == null) {
        matchingTimer = new Timer();
        matchingTimer.start();
        detectionTimers.put(object, matchingTimer);
      }

      if (pendingPoseCount[cameraIndex] < MAX_POSES_PER_CAMERA) {
        pendingPoses[cameraIndex][pendingPoseCount[cameraIndex]] = object.pose();
        pendingPoseCount[cameraIndex]++;
      }

      if (matchingTimer.hasElapsed(ObjectDetectionConstants.MINIMUM_DETECTION_TIME)) {
        activeObjects.add(object);
      }
    }
  }

  private DetectedObject updateConfidence(DetectedObject object, double currentTimestamp) {
    double timeDelta = currentTimestamp - object.lastUpdateTime();
    double newConfidence =
        Math.max(
            ObjectDetectionConstants.MIN_CONFIDENCE,
            object.confidence() - (ObjectDetectionConstants.CONFIDENCE_DECAY_RATE * timeDelta));
    return new DetectedObject(object.pose(), object.classId(), newConfidence, currentTimestamp);
  }

  private boolean isStale(DetectedObject object) {
    return object.confidence() <= ObjectDetectionConstants.MIN_CONFIDENCE;
  }

  private boolean matchesPosition(DetectedObject a, DetectedObject b) {
    double deltaX = a.pose().getX() - b.pose().getX();
    double deltaY = a.pose().getY() - b.pose().getY();
    double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    return distance <= ObjectDetectionConstants.POSITION_MATCH_TOLERANCE;
  }

  private void cleanupTimers() {
    detectionTimers
        .entrySet()
        .removeIf(
            entry -> {
              boolean exceededGracePeriod =
                  entry.getValue().hasElapsed(ObjectDetectionConstants.TIMER_CLEANUP_GRACE_PERIOD);
              boolean isActivelyDetected =
                  activeObjects.stream().anyMatch(obj -> matchesPosition(obj, entry.getKey()));
              return exceededGracePeriod && !isActivelyDetected;
            });
  }

  public List<DetectedObject> getDetectedObjects() {
    return new ArrayList<>(activeObjects);
  }
}
