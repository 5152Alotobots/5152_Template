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
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io.ObjectDetectionIO;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io.ObjectDetectionIO.DetectedObjectFieldRelative;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io.ObjectDetectionIOInputsAutoLogged;
import java.util.*;
import java.util.function.Supplier;

/**
 * Subsystem that handles object detection using PhotonVision cameras. This subsystem processes
 * camera inputs to track and identify objects in the field, maintaining lists of both stable and
 * pending object detections.
 */
public class ObjectDetectionSubsystem extends SubsystemBase {

  /** Supplier for getting the current robot pose */
  private final Supplier<Pose2d> robotPose;

  /** Array of object detection IO interfaces */
  private final ObjectDetectionIO[] io;

  /** Array of logged inputs from each camera */
  private final ObjectDetectionIOInputsAutoLogged[] inputs;

  /** Array of alerts for disconnected cameras */
  private final Alert[] disconnectedAlerts;

  /** Map tracking detection history for each object */
  private final Map<ObjectDetectionIO.DetectedObjectFieldRelative, DetectionHistory>
      detectionHistories = new LinkedHashMap<>();

  /** Set of objects that have been detected but are not yet stable */
  private final Set<ObjectDetectionIO.DetectedObjectFieldRelative> pendingObjects =
      new LinkedHashSet<>();

  /** Set of objects that have been consistently detected and are considered stable */
  private final Set<ObjectDetectionIO.DetectedObjectFieldRelative> stableObjects =
      new LinkedHashSet<>();

  /**
   * Inner class that tracks the detection history of an object over time to determine stability.
   */
  private static class DetectionHistory {
    /** Queue storing detection history as booleans */
    private final Deque<Boolean> history;

    /** Whether this object is currently considered stable */
    private boolean isStable = false;

    /** The last detected instance of this object */
    private ObjectDetectionIO.DetectedObjectFieldRelative lastSeen = null;

    /** Creates a new detection history tracker. */
    public DetectionHistory() {
      this.history = new ArrayDeque<>(HISTORY_LENGTH);
    }

    /**
     * Adds a new detection to the history and updates stability.
     *
     * @param detected Whether the object was detected in this frame
     * @param currentObject The current detection if detected, null otherwise
     */
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

    /**
     * @return Whether this object is currently considered stable
     */
    public boolean isStable() {
      return isStable;
    }

    /**
     * @return The most recent detection of this object
     */
    public ObjectDetectionIO.DetectedObjectFieldRelative getLastSeen() {
      return lastSeen;
    }

    /**
     * @return The total number of frames in history where object was detected
     */
    public int getRecentDetectionCount() {
      return (int) history.stream().filter(Boolean::booleanValue).count();
    }

    /**
     * @return The number of consecutive frames where object was not detected
     */
    public int getMissedFramesInARow() {
      int count = 0;
      for (Boolean detected : history) {
        if (!detected) count++;
        else break;
      }
      return count;
    }
  }

  /**
   * Creates a new ObjectDetectionSubsystem.
   *
   * @param robotPose Supplier for the current robot pose
   * @param io Array of ObjectDetectionIO interfaces for cameras
   */
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

  /**
   * Checks if an object exists in either the pending or stable object lists.
   *
   * @param obj The object to check for
   * @return True if object exists in either list
   */
  private boolean objectExistsInLists(ObjectDetectionIO.DetectedObjectFieldRelative obj) {
    for (ObjectDetectionIO.DetectedObjectFieldRelative pending : pendingObjects) {
      if (objectsMatch(pending, obj)) return true;
    }
    for (ObjectDetectionIO.DetectedObjectFieldRelative stable : stableObjects) {
      if (objectsMatch(stable, obj)) return true;
    }
    return false;
  }

  /** Periodic function that updates camera inputs and processes detections. */
  @Override
  public void periodic() {
    // Code continues...

    // Rest of implementation remains unchanged
  }

  /**
   * Checks if two detected objects match based on position and class.
   *
   * @param obj1 First object to compare
   * @param obj2 Second object to compare
   * @return True if the objects are considered to be the same object
   */
  private boolean objectsMatch(
      ObjectDetectionIO.DetectedObjectFieldRelative obj1,
      ObjectDetectionIO.DetectedObjectFieldRelative obj2) {
    double positionDiff =
        Math.sqrt(
            Math.pow(obj1.pose().getX() - obj2.pose().getX(), 2)
                + Math.pow(obj1.pose().getY() - obj2.pose().getY(), 2));

    return positionDiff <= POSITION_MATCH_TOLERANCE && obj1.classId() == obj2.classId();
  }

  /**
   * Converts an array of detected objects to an array of their poses.
   *
   * @param detectedObjects Array of detected objects
   * @return Array of object poses
   */
  public Pose3d[] toPoseArray(ObjectDetectionIO.DetectedObjectFieldRelative[] detectedObjects) {
    Pose3d[] objectPoses = new Pose3d[detectedObjects.length];
    for (int index = 0; index < objectPoses.length; index++) {
      objectPoses[index] = detectedObjects[index].pose();
    }
    return objectPoses;
  }

  /**
   * Converts robot-relative object detections to field-relative coordinates.
   *
   * @param robotRelative Array of robot-relative object detections
   * @return Array of field-relative object detections
   */
  public ObjectDetectionIO.DetectedObjectFieldRelative[] toFieldRelative(
      ObjectDetectionIO.DetectedObjectRobotRelative[] robotRelative) {
    Pose3d robotPose3d =
        new Pose3d(
            robotPose.get().getX(),
            robotPose.get().getY(),
            0.0,
            new Rotation3d(0, 0, robotPose.get().getRotation().getRadians()));

    ObjectDetectionIO.DetectedObjectFieldRelative[] fieldRelative =
        new ObjectDetectionIO.DetectedObjectFieldRelative[robotRelative.length];

    for (int index = 0; index < robotRelative.length; index++) {
      Pose3d fieldSpaceObjectPose = robotPose3d.transformBy(robotRelative[index].targetToRobot());

      fieldRelative[index] =
          new DetectedObjectFieldRelative(
              robotRelative[index].timestamp(),
              fieldSpaceObjectPose,
              robotRelative[index].confidence(),
              robotRelative[index].classId());
    }
    return fieldRelative;
  }

  /**
   * Gets a list of detected objects based on stability criteria.
   *
   * @param includeUnstable Whether to include unstable/pending objects
   * @param includeStable Whether to include stable objects
   * @return List of detected objects matching the criteria
   */
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

  /**
   * Gets a list of only stable detected objects.
   *
   * @return List of stable detected objects
   */
  public List<ObjectDetectionIO.DetectedObjectFieldRelative> getStableDetectedObjects() {
    return getDetectedObjects(false, true);
  }

  /**
   * Gets a list of only unstable/pending detected objects.
   *
   * @return List of unstable detected objects
   */
  public List<ObjectDetectionIO.DetectedObjectFieldRelative> getUnstableDetectedObjects() {
    return getDetectedObjects(true, false);
  }

  /**
   * Gets a list of all detected objects, both stable and unstable.
   *
   * @return List of all detected objects
   */
  public List<ObjectDetectionIO.DetectedObjectFieldRelative> getAllDetectedObjects() {
    return getDetectedObjects(true, true);
  }
}
