//  ____  _ ____ ____       _    _     ___ _____ ___  ____   ___ _____ ____
// | ___|/ | ___|___ \     / \  | |   / _ \_   _/ _ \| __ ) / _ \_   _/ ___|
// |___ \| |___ \ __) |   / _ \ | |  | | | || || | | |  _ \| | | || | \___ \
//  ___) | |___) / __/   / ___ \| |__| |_| || || |_| | |_) | |_| || |  ___) |
// |____/|_|____/_____| /_/   \_\_____\___/ |_| \___/|____/ \___/ |_| |____/
// 2025 ALOTOBOTS FRC 5152
// Robot Code
package frc.alotobots.library.subsystems.vision.photonvision.objectdetection;

import static frc.alotobots.library.subsystems.vision.photonvision.objectdetection.constants.ObjectDetectionConstants.CAMERA_CONFIGS;
import static frc.alotobots.library.subsystems.vision.photonvision.objectdetection.constants.ObjectDetectionConstants.POSITION_MATCH_TOLERANCE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.constants.ObjectDetectionConstants;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io.ObjectDetectionIO;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io.ObjectDetectionIOInputsAutoLogged;
import java.util.*;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ObjectDetectionSubsystem extends SubsystemBase {
  private static final int HISTORY_LENGTH = 20;
  private static final int REQUIRED_DETECTIONS = 3;
  private static final int MISSING_FRAMES_THRESHOLD = 10;

  private final Supplier<Pose2d> robotPose;
  private final ObjectDetectionIO[] io;
  private final ObjectDetectionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private final Map<ObjectDetectionIO.DetectedObject, DetectionHistory> detectionHistories =
      new LinkedHashMap<>();
  private final Set<ObjectDetectionIO.DetectedObject> pendingObjects = new LinkedHashSet<>();
  private final Set<ObjectDetectionIO.DetectedObject> stableObjects = new LinkedHashSet<>();

  private static class DetectionHistory {
    private final LinkedList<Boolean> history = new LinkedList<>();
    private boolean isStable = false;
    private ObjectDetectionIO.DetectedObject lastSeen = null;

    public void addDetection(boolean detected, ObjectDetectionIO.DetectedObject currentObject) {
      history.addFirst(detected);
      if (detected) {
        lastSeen = currentObject;
      }
      if (history.size() > HISTORY_LENGTH) {
        history.removeLast();
      }

      if (!isStable && getRecentDetectionCount() >= REQUIRED_DETECTIONS) {
        isStable = true;
      } else if (isStable && getMissedFramesInARow() > MISSING_FRAMES_THRESHOLD) {
        isStable = false;
      }
    }

    public boolean isStable() {
      return isStable;
    }

    public ObjectDetectionIO.DetectedObject getLastSeen() {
      return lastSeen;
    }

    public int getRecentDetectionCount() {
      int count = 0;
      for (Boolean detected : history) {
        if (detected) count++;
      }
      return count;
    }

    public int getMissedFramesInARow() {
      int count = 0;
      for (Boolean detected : history) {
        if (!detected) count++;
        else break;
      }
      return count;
    }
  }

  public ObjectDetectionSubsystem(Supplier<Pose2d> robotPose, ObjectDetectionIO... io) {
    this.robotPose = robotPose;
    this.io = io;

    this.inputs = new ObjectDetectionIOInputsAutoLogged[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new ObjectDetectionIOInputsAutoLogged();
    }

    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + CAMERA_CONFIGS[i].name() + " is disconnected.",
              Alert.AlertType.kWarning);
    }
  }

  /** Checks if an object already exists in either the pending or stable lists */
  private boolean objectExistsInLists(ObjectDetectionIO.DetectedObject obj) {
    for (ObjectDetectionIO.DetectedObject pending : pendingObjects) {
      if (objectsMatch(pending, obj)) return true;
    }
    for (ObjectDetectionIO.DetectedObject stable : stableObjects) {
      if (objectsMatch(stable, obj)) return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // Update camera inputs
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs(
          "Vision/ObjectDetection/Camera" + ObjectDetectionConstants.CAMERA_CONFIGS[i].name(),
          inputs[i]);
    }

    // Process each camera
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      Set<ObjectDetectionIO.DetectedObject> currentFrameObjects =
          new LinkedHashSet<>(List.of(inputs[cameraIndex].detectedObjects));
      Set<ObjectDetectionIO.DetectedObject> trackedObjectsSeen = new LinkedHashSet<>();

      // Update existing tracked objects
      Iterator<Map.Entry<ObjectDetectionIO.DetectedObject, DetectionHistory>> it =
          detectionHistories.entrySet().iterator();
      while (it.hasNext()) {
        Map.Entry<ObjectDetectionIO.DetectedObject, DetectionHistory> entry = it.next();
        ObjectDetectionIO.DetectedObject trackedObject = entry.getKey();
        DetectionHistory history = entry.getValue();
        boolean wasStable = history.isStable();

        boolean stillDetected = false;
        ObjectDetectionIO.DetectedObject currentMatchedObject = null;

        for (ObjectDetectionIO.DetectedObject currentObject : currentFrameObjects) {
          if (objectsMatch(trackedObject, currentObject)) {
            stillDetected = true;
            currentMatchedObject = currentObject;
            trackedObjectsSeen.add(currentObject);
            break;
          }
        }

        history.addDetection(stillDetected, currentMatchedObject);
        
        // Handle state changes
        if (stillDetected && currentMatchedObject != null) {
          if (history.isStable() && !wasStable) {
            // Object became stable - remove from pending if present, add to stable if not already
            // there
            pendingObjects.remove(trackedObject);
            if (!objectExistsInLists(currentMatchedObject)) {
              stableObjects.add(currentMatchedObject);
            }
          } else if (!history.isStable() && wasStable) {
            // Object lost stability - remove from stable, add to pending if not already there
            stableObjects.remove(trackedObject);
            if (!objectExistsInLists(currentMatchedObject)) {
              pendingObjects.add(currentMatchedObject);
            }
          } else {
            // Update position if needed
            if (wasStable) {
              if (stableObjects.remove(trackedObject)
                  && !objectExistsInLists(currentMatchedObject)) {
                stableObjects.add(currentMatchedObject);
              }
            } else {
              if (pendingObjects.remove(trackedObject)
                  && !objectExistsInLists(currentMatchedObject)) {
                pendingObjects.add(currentMatchedObject);
              }
            }
          }
        }

        // Remove if missing too long
        if (history.getMissedFramesInARow() > MISSING_FRAMES_THRESHOLD) {
          it.remove();
          pendingObjects.remove(trackedObject);
          stableObjects.remove(trackedObject);
        }
      }

      // Add new objects to tracking
      for (ObjectDetectionIO.DetectedObject detectedObject : currentFrameObjects) {
        if (!trackedObjectsSeen.contains(detectedObject) && !objectExistsInLists(detectedObject)) {
          DetectionHistory history = new DetectionHistory();
          history.addDetection(true, detectedObject);
          detectionHistories.put(detectedObject, history);
          pendingObjects.add(detectedObject);
        }
      }

      // Update logging for this camera
      Logger.recordOutput(
          "Vision/ObjectDetection/Camera"
              + CAMERA_CONFIGS[cameraIndex].name()
              + "/RobotRelative/PendingObjects",
          pendingObjects.toArray(new ObjectDetectionIO.DetectedObject[0]));
      Logger.recordOutput(
          "Vision/ObjectDetection/Camera"
              + CAMERA_CONFIGS[cameraIndex].name()
              + "/RobotRelative/StableObjects",
          stableObjects.toArray(new ObjectDetectionIO.DetectedObject[0]));
      Logger.recordOutput(
          "Vision/ObjectDetection/Camera"
              + CAMERA_CONFIGS[cameraIndex].name()
              + "/FieldRelative/PendingObjects",
          toFieldRelative(pendingObjects.toArray(new ObjectDetectionIO.DetectedObject[0])));
      Logger.recordOutput(
          "Vision/ObjectDetection/Camera"
              + CAMERA_CONFIGS[cameraIndex].name()
              + "/FieldRelative/StableObjects",
          toFieldRelative(stableObjects.toArray(new ObjectDetectionIO.DetectedObject[0])));
    }

    // Log summary
    Logger.recordOutput(
        "Vision/ObjectDetection/Summary/RobotRelative/PendingObjects",
        pendingObjects.toArray(new ObjectDetectionIO.DetectedObject[0]));
    Logger.recordOutput(
        "Vision/ObjectDetection/Summary/RobotRelative/StableObjects",
        stableObjects.toArray(new ObjectDetectionIO.DetectedObject[0]));
    Logger.recordOutput(
        "Vision/ObjectDetection/Summary/FieldRelative/PendingObjects",
        toFieldRelative(pendingObjects.toArray(new ObjectDetectionIO.DetectedObject[0])));
    Logger.recordOutput(
        "Vision/ObjectDetection/Summary/FieldRelative/StableObjects",
        toFieldRelative(stableObjects.toArray(new ObjectDetectionIO.DetectedObject[0])));
  }

  private boolean objectsMatch(
      ObjectDetectionIO.DetectedObject obj1, ObjectDetectionIO.DetectedObject obj2) {
    double positionDiff =
        Math.sqrt(
            Math.pow(obj1.targetToRobot().getX() - obj2.targetToRobot().getX(), 2)
                + Math.pow(obj1.targetToRobot().getY() - obj2.targetToRobot().getY(), 2));
    if (positionDiff > POSITION_MATCH_TOLERANCE) {
      return false;
    }

    if (obj1.classId() != obj2.classId()) {
      return false;
    }

    return true;
  }

  private Pose3d toFieldRelative(ObjectDetectionIO.DetectedObject detectedObject) {
    Pose3d robotPose3d =
        new Pose3d(
            robotPose.get().getX(),
            robotPose.get().getY(),
            0.0,
            new Rotation3d(0, 0, robotPose.get().getRotation().getRadians()));
    return robotPose3d.transformBy(detectedObject.targetToRobot());
  }

  private Pose3d[] toFieldRelative(ObjectDetectionIO.DetectedObject[] detectedObjects) {
    LinkedList<Pose3d> fieldRelativePoses = new LinkedList<>();
    for (ObjectDetectionIO.DetectedObject detectedObject : detectedObjects) {
      fieldRelativePoses.add(toFieldRelative(detectedObject));
    }
    return fieldRelativePoses.toArray(new Pose3d[fieldRelativePoses.size()]);
  }
}
