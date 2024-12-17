//   ____  _ ____ ____       _    _     ___ _____ ___  ____   ___ _____ ____
//  | ___|/ | ___|___      /   | |   / _ _   _/ _ | __ ) / _ _   _/ ___|
//  |___ | |___  __) |   / _  | |  | | | || || | | |  _ | | | || | ___ //   ___) | |___) / __/   /
// ___ | |__| |_| || || |_| | |_) | |_| || |  ___) |
//  |____/|_|____/_____| /_/   _________/ |_| ___/|____/ ___/ |_| |____/
//
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
  private static final int HISTORY_LENGTH = 300;
  private static final int REQUIRED_DETECTIONS = 150;
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

      // Use total detections for stability gain
      int totalDetections = getRecentDetectionCount();

      if (!isStable && totalDetections >= REQUIRED_DETECTIONS) {
        isStable = true;
        System.out.println(
            "Object became stable with " + totalDetections + " total detections in history");
      } else if (isStable && getMissedFramesInARow() > MISSING_FRAMES_THRESHOLD) {
        isStable = false;
        System.out.println(
            "Object lost stability after "
                + getMissedFramesInARow()
                + " consecutive missed frames");
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

        if (stillDetected && currentMatchedObject != null) {
          if (history.isStable() && !wasStable) {
            // Object became stable
            System.out.println("Attempting to move object to stable list");
            System.out.println(
                "Before move - Pending size: "
                    + pendingObjects.size()
                    + ", Stable size: "
                    + stableObjects.size());

            ObjectDetectionIO.DetectedObject toRemove = null;
            for (ObjectDetectionIO.DetectedObject pending : pendingObjects) {
              if (objectsMatch(pending, trackedObject)) {
                toRemove = pending;
                break;
              }
            }

            if (toRemove != null) {
              pendingObjects.remove(toRemove);
              stableObjects.add(currentMatchedObject);
              System.out.println("Successfully moved object from pending to stable");
            } else {
              System.out.println("Failed to find matching object in pending list");
            }

            System.out.println(
                "After move - Pending size: "
                    + pendingObjects.size()
                    + ", Stable size: "
                    + stableObjects.size());
          } else if (!history.isStable() && wasStable) {
            // Object lost stability
            System.out.println("Attempting to remove unstable object from stable list");
            System.out.println("Before remove - Stable size: " + stableObjects.size());

            ObjectDetectionIO.DetectedObject toRemove = null;
            for (ObjectDetectionIO.DetectedObject stable : stableObjects) {
              if (objectsMatch(stable, trackedObject)) {
                toRemove = stable;
                break;
              }
            }

            if (toRemove != null) {
              stableObjects.remove(toRemove);
              pendingObjects.add(currentMatchedObject);
              System.out.println("Successfully moved unstable object back to pending");
            } else {
              System.out.println("Failed to find matching object in stable list");
            }

            System.out.println("After remove - Stable size: " + stableObjects.size());
          }
        }

        // Remove if missing too long
        if (history.getMissedFramesInARow() > MISSING_FRAMES_THRESHOLD) {
          System.out.println("Removing object that's been missing too long");
          System.out.println(
              "Before removal - Pending size: "
                  + pendingObjects.size()
                  + ", Stable size: "
                  + stableObjects.size());

          // Remove from histories
          it.remove();

          // Find and remove from stable objects if present
          ObjectDetectionIO.DetectedObject toRemoveStable = null;
          for (ObjectDetectionIO.DetectedObject stable : stableObjects) {
            if (objectsMatch(stable, trackedObject)) {
              toRemoveStable = stable;
              break;
            }
          }
          if (toRemoveStable != null) {
            stableObjects.remove(toRemoveStable);
            System.out.println("Removed from stable objects");
          }

          // Find and remove from pending objects if present
          ObjectDetectionIO.DetectedObject toRemovePending = null;
          for (ObjectDetectionIO.DetectedObject pending : pendingObjects) {
            if (objectsMatch(pending, trackedObject)) {
              toRemovePending = pending;
              break;
            }
          }
          if (toRemovePending != null) {
            pendingObjects.remove(toRemovePending);
            System.out.println("Removed from pending objects");
          }

          System.out.println(
              "After removal - Pending size: "
                  + pendingObjects.size()
                  + ", Stable size: "
                  + stableObjects.size());
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
    System.out.println("Comparing objects:");
    System.out.println(
        "Obj1: x="
            + obj1.targetToRobot().getX()
            + " y="
            + obj1.targetToRobot().getY()
            + " class="
            + obj1.classId());
    System.out.println(
        "Obj2: x="
            + obj2.targetToRobot().getX()
            + " y="
            + obj2.targetToRobot().getY()
            + " class="
            + obj2.classId());

    double positionDiff =
        Math.sqrt(
            Math.pow(obj1.targetToRobot().getX() - obj2.targetToRobot().getX(), 2)
                + Math.pow(obj1.targetToRobot().getY() - obj2.targetToRobot().getY(), 2));
    System.out.println(
        "Position difference: " + positionDiff + " (tolerance: " + POSITION_MATCH_TOLERANCE + ")");

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
    return fieldRelativePoses.toArray(new Pose3d[0]);
  }
}
