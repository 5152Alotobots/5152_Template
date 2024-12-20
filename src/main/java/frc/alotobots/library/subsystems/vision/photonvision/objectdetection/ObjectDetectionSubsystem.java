/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2024 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.alotobots.library.subsystems.vision.photonvision.objectdetection;

import static frc.alotobots.library.subsystems.vision.photonvision.objectdetection.constants.ObjectDetectionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.constants.ObjectDetectionConstants;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io.ObjectDetectionIO;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io.ObjectDetectionIO.DetectedObjectFieldRelative;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io.ObjectDetectionIOInputsAutoLogged;
import java.util.*;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ObjectDetectionSubsystem extends SubsystemBase {

  private final Supplier<Pose2d> robotPose;
  private final ObjectDetectionIO[] io;
  private final ObjectDetectionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private final Map<ObjectDetectionIO.DetectedObjectFieldRelative, DetectionHistory>
      detectionHistories = new LinkedHashMap<>();
  private final Set<ObjectDetectionIO.DetectedObjectFieldRelative> pendingObjects =
      new LinkedHashSet<>();
  private final Set<ObjectDetectionIO.DetectedObjectFieldRelative> stableObjects =
      new LinkedHashSet<>();

  private static class DetectionHistory {
    private final Deque<Boolean> history;
    private boolean isStable = false;
    private ObjectDetectionIO.DetectedObjectFieldRelative lastSeen = null;

    public DetectionHistory() {
      this.history = new ArrayDeque<>(HISTORY_LENGTH);
    }

    public void addDetection(
        boolean detected, ObjectDetectionIO.DetectedObjectFieldRelative currentObject) {
      if (history.size() >= HISTORY_LENGTH) {
        history.removeLast();
      }
      history.addFirst(detected);

      if (detected) {
        lastSeen = currentObject;
      }

      int totalDetections = getRecentDetectionCount();

      if (!isStable && totalDetections >= REQUIRED_DETECTIONS) {
        isStable = true;
      } else if (isStable && getMissedFramesInARow() > MISSING_FRAMES_THRESHOLD) {
        isStable = false;
      }
    }

    public boolean isStable() {
      return isStable;
    }

    public ObjectDetectionIO.DetectedObjectFieldRelative getLastSeen() {
      return lastSeen;
    }

    public int getRecentDetectionCount() {
      return (int) history.stream().filter(Boolean::booleanValue).count();
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

  private boolean objectExistsInLists(ObjectDetectionIO.DetectedObjectFieldRelative obj) {
    for (ObjectDetectionIO.DetectedObjectFieldRelative pending : pendingObjects) {
      if (objectsMatch(pending, obj)) return true;
    }
    for (ObjectDetectionIO.DetectedObjectFieldRelative stable : stableObjects) {
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

      Set<ObjectDetectionIO.DetectedObjectFieldRelative> currentFrameObjects =
          new LinkedHashSet<>(List.of(toFieldRelative(inputs[cameraIndex].detectedObjects)));
      Set<ObjectDetectionIO.DetectedObjectFieldRelative> trackedObjectsSeen = new LinkedHashSet<>();

      // Update existing tracked objects
      Iterator<Map.Entry<ObjectDetectionIO.DetectedObjectFieldRelative, DetectionHistory>> it =
          detectionHistories.entrySet().iterator();
      while (it.hasNext()) {
        Map.Entry<ObjectDetectionIO.DetectedObjectFieldRelative, DetectionHistory> entry =
            it.next();
        ObjectDetectionIO.DetectedObjectFieldRelative trackedObject = entry.getKey();
        DetectionHistory history = entry.getValue();
        boolean wasStable = history.isStable();

        boolean stillDetected = false;
        ObjectDetectionIO.DetectedObjectFieldRelative currentMatchedObject = null;

        for (ObjectDetectionIO.DetectedObjectFieldRelative currentObject : currentFrameObjects) {
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
            ObjectDetectionIO.DetectedObjectFieldRelative toRemove = null;
            for (ObjectDetectionIO.DetectedObjectFieldRelative pending : pendingObjects) {
              if (objectsMatch(pending, trackedObject)) {
                toRemove = pending;
                break;
              }
            }

            if (toRemove != null) {
              pendingObjects.remove(toRemove);
              stableObjects.add(currentMatchedObject);
            }
          } else if (!history.isStable() && wasStable) {
            // Object lost stability
            ObjectDetectionIO.DetectedObjectFieldRelative toRemove = null;
            for (ObjectDetectionIO.DetectedObjectFieldRelative stable : stableObjects) {
              if (objectsMatch(stable, trackedObject)) {
                toRemove = stable;
                break;
              }
            }

            if (toRemove != null) {
              stableObjects.remove(toRemove);
              pendingObjects.add(currentMatchedObject);
            }
          }
        }

        // Remove if missing too long
        if (history.getMissedFramesInARow() > MISSING_FRAMES_THRESHOLD) {
          // Remove from histories
          it.remove();

          // Find and remove from stable objects if present
          ObjectDetectionIO.DetectedObjectFieldRelative toRemoveStable = null;
          for (ObjectDetectionIO.DetectedObjectFieldRelative stable : stableObjects) {
            if (objectsMatch(stable, trackedObject)) {
              toRemoveStable = stable;
              break;
            }
          }
          if (toRemoveStable != null) {
            stableObjects.remove(toRemoveStable);
          }

          // Find and remove from pending objects if present
          ObjectDetectionIO.DetectedObjectFieldRelative toRemovePending = null;
          for (ObjectDetectionIO.DetectedObjectFieldRelative pending : pendingObjects) {
            if (objectsMatch(pending, trackedObject)) {
              toRemovePending = pending;
              break;
            }
          }
          if (toRemovePending != null) {
            pendingObjects.remove(toRemovePending);
          }
        }
      }

      // Add new objects to tracking
      for (ObjectDetectionIO.DetectedObjectFieldRelative detectedObject : currentFrameObjects) {
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
          pendingObjects.toArray(new ObjectDetectionIO.DetectedObjectFieldRelative[0]));
      Logger.recordOutput(
          "Vision/ObjectDetection/Camera"
              + CAMERA_CONFIGS[cameraIndex].name()
              + "/RobotRelative/StableObjects",
          stableObjects.toArray(new ObjectDetectionIO.DetectedObjectFieldRelative[0]));
      Logger.recordOutput(
          "Vision/ObjectDetection/Camera"
              + CAMERA_CONFIGS[cameraIndex].name()
              + "/FieldRelative/PendingObjects",
          pendingObjects.toArray(new ObjectDetectionIO.DetectedObjectFieldRelative[0]));
      Logger.recordOutput(
          "Vision/ObjectDetection/Camera"
              + CAMERA_CONFIGS[cameraIndex].name()
              + "/FieldRelative/StableObjects",
          stableObjects.toArray(new ObjectDetectionIO.DetectedObjectFieldRelative[0]));
    }

    // Log summary
    Logger.recordOutput(
        "Vision/ObjectDetection/Summary/RobotRelative/PendingObjects",
        pendingObjects.toArray(new ObjectDetectionIO.DetectedObjectFieldRelative[0]));
    Logger.recordOutput(
        "Vision/ObjectDetection/Summary/RobotRelative/StableObjects",
        stableObjects.toArray(new ObjectDetectionIO.DetectedObjectFieldRelative[0]));
    Logger.recordOutput(
        "Vision/ObjectDetection/Summary/FieldRelative/PendingObjects",
        pendingObjects.toArray(new ObjectDetectionIO.DetectedObjectFieldRelative[0]));
    Logger.recordOutput(
        "Vision/ObjectDetection/Summary/FieldRelative/StableObjects",
        stableObjects.toArray(new ObjectDetectionIO.DetectedObjectFieldRelative[0]));
  }

  private boolean objectsMatch(
      ObjectDetectionIO.DetectedObjectFieldRelative obj1,
      ObjectDetectionIO.DetectedObjectFieldRelative obj2) {
    double positionDiff =
        Math.sqrt(
            Math.pow(obj1.pose().getX() - obj2.pose().getX(), 2)
                + Math.pow(obj1.pose().getY() - obj2.pose().getY(), 2));

    return positionDiff <= POSITION_MATCH_TOLERANCE && obj1.classId() == obj2.classId();
  }

  public Pose3d[] toPoseArray(ObjectDetectionIO.DetectedObjectFieldRelative[] detectedObjects) {
    // loop through each detection, putting them into an array
    Pose3d[] objectPoses = new Pose3d[detectedObjects.length];
    for (int index = 0; index < objectPoses.length; index++) {
      objectPoses[index] = detectedObjects[index].pose();
    }
    return objectPoses;
  }

  public ObjectDetectionIO.DetectedObjectFieldRelative[] toFieldRelative(
      ObjectDetectionIO.DetectedObjectRobotRelative[] robotRelative) {
    // Convert robot pose to Pose3d
    Pose3d robotPose3d =
        new Pose3d(
            robotPose.get().getX(),
            robotPose.get().getY(),
            0.0, // Assume robot is on the ground
            new Rotation3d() // If we need to account for rotation, change this!
            );

    // Make new array to hold field relative objects
    ObjectDetectionIO.DetectedObjectFieldRelative[] fieldRelative =
        new ObjectDetectionIO.DetectedObjectFieldRelative[robotRelative.length];

    // Loop through the originl array
    for (int index = 0; index < robotRelative.length; index++) {
      // Transform to field space
      Pose3d fieldSpaceObjectPose = robotPose3d.transformBy(robotRelative[index].targetToRobot());

      // Asign to new, field relative array
      fieldRelative[index] =
          new DetectedObjectFieldRelative(
              robotRelative[index].timestamp(),
              fieldSpaceObjectPose,
              robotRelative[index].confidence(),
              robotRelative[index].classId());
    }
    return fieldRelative;
  }

  public List<ObjectDetectionIO.DetectedObjectFieldRelative> getDetectedObjects(
      boolean includeUnstable, boolean includeStable) {
    List<ObjectDetectionIO.DetectedObjectFieldRelative> detectedObjects = new ArrayList<>();
    if (includeStable) {
      detectedObjects.addAll(stableObjects);
    }
    if (includeUnstable) {
      detectedObjects.addAll(pendingObjects);
    }
    return detectedObjects;
  }

  public List<ObjectDetectionIO.DetectedObjectFieldRelative> getStableDetectedObjects() {
    return getDetectedObjects(false, true);
  }

  public List<ObjectDetectionIO.DetectedObjectFieldRelative> getUnstableDetectedObjects() {
    return getDetectedObjects(true, false);
  }

  public List<ObjectDetectionIO.DetectedObjectFieldRelative> getAllDetectedObjects() {
    return getDetectedObjects(true, true);
  }
}
