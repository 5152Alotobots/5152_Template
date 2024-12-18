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
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io.ObjectDetectionIOInputsAutoLogged;
import java.util.*;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ObjectDetectionSubsystem extends SubsystemBase {

  private final Supplier<Pose2d> robotPose;
  private final ObjectDetectionIO[] io;
  private final ObjectDetectionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private final Map<ObjectDetectionIO.DetectedObject, DetectionHistory> detectionHistories =
      new LinkedHashMap<>();
  private final Set<ObjectDetectionIO.DetectedObject> pendingObjects = new LinkedHashSet<>();
  private final Set<ObjectDetectionIO.DetectedObject> stableObjects = new LinkedHashSet<>();
  private final Map<ObjectDetectionIO.DetectedObject, Pose3d> fieldPositionCache =
      new LinkedHashMap<>();

  private static class DetectionHistory {
    private final Deque<Boolean> history;
    private boolean isStable = false;
    private ObjectDetectionIO.DetectedObject lastSeen = null;

    public DetectionHistory() {
      this.history = new ArrayDeque<>(HISTORY_LENGTH);
    }

    public void addDetection(boolean detected, ObjectDetectionIO.DetectedObject currentObject) {
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

    public ObjectDetectionIO.DetectedObject getLastSeen() {
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
            }
          } else if (!history.isStable() && wasStable) {
            // Object lost stability
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
            }
          }
        }

        // Remove if missing too long
        if (history.getMissedFramesInARow() > MISSING_FRAMES_THRESHOLD) {
          // Remove from histories and cache
          it.remove();
          fieldPositionCache.remove(trackedObject);

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
          }
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

    return positionDiff <= POSITION_MATCH_TOLERANCE && obj1.classId() == obj2.classId();
  }

  public Pose3d toFieldRelative(ObjectDetectionIO.DetectedObject detectedObject) {
    // If we already have a cached position, use it
    if (fieldPositionCache.containsKey(detectedObject)) {
      return fieldPositionCache.get(detectedObject);
    }

    // Calculate field position for new objects
    Pose2d robotPose2d = robotPose.get();
    Pose3d robotPose3d =
        new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0.0,
            new Rotation3d(0, 0, robotPose2d.getRotation().getRadians()));

    Pose3d fieldPosition = robotPose3d.transformBy(detectedObject.targetToRobot());

    // Cache the position
    fieldPositionCache.put(detectedObject, fieldPosition);

    return fieldPosition;
  }

  public Pose3d[] toFieldRelative(ObjectDetectionIO.DetectedObject[] detectedObjects) {
    List<Pose3d> fieldRelativePoses = new ArrayList<>();
    for (ObjectDetectionIO.DetectedObject detectedObject : detectedObjects) {
      fieldRelativePoses.add(toFieldRelative(detectedObject));
    }
    return fieldRelativePoses.toArray(new Pose3d[0]);
  }

  // DO NOT USE TRANSLATION FOR ANY MOVEMENT, IT IS TIED TO THE ROBOT SPACE AND NOT FIELD SPACE!
  public List<ObjectDetectionIO.DetectedObject> getDetectedObjects(boolean includeUnstable) {
    List<ObjectDetectionIO.DetectedObject> detectedObjects = new ArrayList<>();
    detectedObjects.addAll(stableObjects);
    if (includeUnstable) {
      detectedObjects.addAll(pendingObjects);
    }
    return detectedObjects;
  }

  // DO NOT USE TRANSLATION FOR ANY MOVEMENT, IT IS TIED TO THE ROBOT SPACE AND NOT FIELD SPACE!
  public List<ObjectDetectionIO.DetectedObject> getStableDetectedObjects() {
    return getDetectedObjects(false);
  }

  // DO NOT USE TRANSLATION FOR ANY MOVEMENT, IT IS TIED TO THE ROBOT SPACE AND NOT FIELD SPACE!
  public List<ObjectDetectionIO.DetectedObject> getAllDetectedObjects() {
    return getDetectedObjects(true);
  }

  public List<Pose3d> getFieldRelativePoses(boolean includeUnstable) {
    List<ObjectDetectionIO.DetectedObject> objects = getDetectedObjects(includeUnstable);
    List<Pose3d> poses = new ArrayList<>();
    for (ObjectDetectionIO.DetectedObject object : objects) {
      poses.add(toFieldRelative(object));
    }
    return poses;
  }

  public List<Pose3d> getStableFieldRelativePoses() {
    return getFieldRelativePoses(false);
  }

  public List<Pose3d> getAllFieldRelativePoses() {
    return getFieldRelativePoses(true);
  }
}
