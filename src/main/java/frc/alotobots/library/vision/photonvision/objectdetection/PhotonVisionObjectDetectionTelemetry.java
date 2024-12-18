package frc.alotobots.library.vision.photonvision.objectdetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.photonvision.PhotonCamera;

public class PhotonVisionObjectDetectionTelemetry {
  private final ShuffleboardTab tab;
  private final Field2d field;
  private final List<CameraWidget> cameraWidgets = new ArrayList<>();
  private final ShuffleboardLayout globalStatsList;
  private final GenericEntry totalObjectsEntry;
  private GenericEntry objectDetectionEnabled;
  private GenericEntry teleopOnlyEnabled;
  private final ShuffleboardLayout objectsList;
  private final List<GenericEntry> objectEntries;

  private static class CameraWidget {
    final ShuffleboardLayout layout;
    final GenericEntry connectionStatus;
    final GenericEntry enabledEntry;
    final GenericEntry distanceEntry;
    final GenericEntry poseXEntry;
    final GenericEntry poseYEntry;
    final GenericEntry targetIdEntry;
    final GenericEntry confidenceEntry;

    CameraWidget(ShuffleboardTab tab, String cameraName, int position) {
      layout =
          tab.getLayout("Camera " + cameraName, BuiltInLayouts.kList)
              .withSize(2, 3)
              .withPosition(position, 4)
              .withProperties(Map.of("Label position", "LEFT"));

      connectionStatus = layout.add("Connected", false).getEntry();
      enabledEntry =
          layout.add("Enabled", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
      distanceEntry = layout.add("Target Distance (m)", 0.0).getEntry();
      poseXEntry = layout.add("Target X", 0.0).getEntry();
      poseYEntry = layout.add("Target Y", 0.0).getEntry();
      targetIdEntry = layout.add("Target ID", -1).getEntry();
      confidenceEntry = layout.add("Confidence", 0.0).getEntry();
    }
  }

  /**
   * Gets the current state of the object detection enabled toggle.
   *
   * @return true if object detection is enabled, false otherwise
   */
  public boolean isObjectDetectionEnabled() {
    return objectDetectionEnabled.getBoolean(
        PhotonVisionObjectDetectionSubsystemConstants.USE_OBJECT_DETECTION);
  }

  /**
   * Gets the current state of the teleop-only toggle.
   *
   * @return true if teleop-only mode is enabled, false otherwise
   */
  public boolean isTeleopOnlyEnabled() {
    return teleopOnlyEnabled.getBoolean(
        PhotonVisionObjectDetectionSubsystemConstants.ONLY_USE_OBJECT_DETECTION_IN_TELEOP);
  }

  /**
   * Checks if a camera is enabled in the telemetry interface.
   *
   * @param cameraIndex The index of the camera to check
   * @return true if the camera is enabled, false otherwise
   */
  public boolean isCameraEnabled(int cameraIndex) {
    if (cameraIndex >= 0 && cameraIndex < cameraWidgets.size()) {
      return cameraWidgets.get(cameraIndex).enabledEntry.getBoolean(true);
    }
    return false;
  }

  /**
   * Draws tracer lines from robot to each detected object.
   *
   * @param objects List of detected objects to draw lines to
   */
  private void drawTracerLine(DetectedObject obj, int objIndex) {
    if (obj != null && obj.getPose() != null && obj.getDrive() != null) {
      var robotPose = obj.getDrive().getState().Pose;
      var targetPose = obj.getPose().toPose2d();

      // Create trajectory points for the line
      List<Trajectory.State> states = new ArrayList<>();
      for (int i = 0; i <= 8; i++) {
        double t = i / 8.0;
        double x = robotPose.getX() + (targetPose.getX() - robotPose.getX()) * t;
        double y = robotPose.getY() + (targetPose.getY() - robotPose.getY()) * t;
        states.add(new Trajectory.State(t, 0, 0, new Pose2d(x, y, robotPose.getRotation()), 0));
      }
      Trajectory line = new Trajectory(states);

      // Add the line to the field with a unique name for this object
      String lineName = "TracerLine" + objIndex;
      field.getObject(lineName).setTrajectory(line);
    }
  }

  public PhotonVisionObjectDetectionTelemetry() {
    tab = Shuffleboard.getTab("Object Detection");

    // Initialize global stats widget
    globalStatsList =
        tab.getLayout("Global Stats", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withPosition(0, 0)
            .withProperties(Map.of("Label position", "LEFT"));

    totalObjectsEntry = globalStatsList.add("Total Objects", 0).getEntry();

    // Initialize detected objects list widget
    objectsList =
        tab.getLayout("Detected Objects", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(8, 0)
            .withProperties(Map.of("Label position", "LEFT"));

    // Initialize empty list for object entries
    objectEntries = new ArrayList<>();

    // Add global settings with persistent toggle switches
    this.objectDetectionEnabled =
        tab.add(
                "Object Detection Enabled",
                PhotonVisionObjectDetectionSubsystemConstants.USE_OBJECT_DETECTION)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(0, 2)
            .withSize(2, 1)
            .getEntry();

    this.teleopOnlyEnabled =
        tab.add(
                "Only Use in Teleop",
                PhotonVisionObjectDetectionSubsystemConstants.ONLY_USE_OBJECT_DETECTION_IN_TELEOP)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(0, 3)
            .withSize(2, 1)
            .getEntry();

    // Initialize camera widgets
    // Initialize camera widgets - position them below the field
    for (int i = 0; i < PhotonVisionObjectDetectionSubsystemConstants.CAMERAS.length; i++) {
      PhotonCamera camera = PhotonVisionObjectDetectionSubsystemConstants.CAMERAS[i];
      if (camera != null) {
        cameraWidgets.add(new CameraWidget(tab, camera.getName(), i * 2));
      }
    }

    // Initialize Field2d widget
    field = new Field2d();
    tab.add("Field", field).withPosition(2, 0).withSize(6, 4);
  }

  public void updateObjects(List<DetectedObject> objects) {
    // Update global stats
    totalObjectsEntry.setDouble(objects.size());

    // Update entries for up to top 3 detected objects
    int numToShow = Math.min(objects.size(), 3);
    for (int i = 0; i < numToShow; i++) {
      DetectedObject obj = objects.get(i);
      // Ensure we have exactly 9 entries (3 objects * 3 values each)
      while (objectEntries.size() < 9) {
        objectEntries.add(
            objectsList.add("Object " + (objectEntries.size() / 3) + " X", 0.0).getEntry());
        objectEntries.add(
            objectsList.add("Object " + (objectEntries.size() / 3) + " Y", 0.0).getEntry());
        objectEntries.add(
            objectsList
                .add("Object " + (objectEntries.size() / 3) + " Confidence", 0.0)
                .getEntry());
      }

      objectEntries.get(i * 3).setDouble(truncate(obj.getPose().getX(), 2));
      objectEntries.get(i * 3 + 1).setDouble(truncate(obj.getPose().getY(), 2));
      objectEntries.get(i * 3 + 2).setDouble(truncate(obj.getConfidence(), 3));
    }

    // Always update robot pose if drive is available
    if (!objects.isEmpty() && objects.get(0) != null && objects.get(0).getDrive() != null) {
      field.setRobotPose(objects.get(0).getDrive().getState().Pose);
    }

    // Update detected objects on field
    if (!objects.isEmpty()) {
      // Plot each detected object
      for (int i = 0; i < objects.size(); i++) {
        DetectedObject obj = objects.get(i);
        if (obj != null && obj.getPose() != null) {
          // Set the object's pose on the field
          field.getObject("Target" + i).setPose(obj.getPose().toPose2d());

          // Draw tracer line from robot to this object
          drawTracerLine(obj, i);
        }
      }
    } else {
      // Clear all objects from field when no objects are detected
      for (int i = 0; i < 10; i++) {
        field.getObject("Target" + i).setPose(new Pose2d());
        // Clear any tracer lines
        field.getObject("TracerLine" + i).setTrajectory(new Trajectory());
      }

      // Clear all existing entry values
      for (int i = 0; i < objectEntries.size(); i++) {
        objectEntries.get(i).setDouble(0.0);
      }
    }

    // Update camera widgets
    for (int i = 0; i < PhotonVisionObjectDetectionSubsystemConstants.CAMERAS.length; i++) {
      PhotonCamera camera = PhotonVisionObjectDetectionSubsystemConstants.CAMERAS[i];
      if (camera != null && i < cameraWidgets.size()) {
        CameraWidget widget = cameraWidgets.get(i);

        // Update connection status
        widget.connectionStatus.setBoolean(camera.isConnected());

        // Find corresponding detected object for this camera
        DetectedObject cameraObject = null;
        for (DetectedObject obj : objects) {
          if (obj != null && obj.getTarget() != null) {
            cameraObject = obj;
            break;
          }
        }

        if (cameraObject != null) {
          widget.distanceEntry.setDouble(truncate(cameraObject.getDistance(), 3));
          widget.poseXEntry.setDouble(truncate(cameraObject.getPose().getX(), 3));
          widget.poseYEntry.setDouble(truncate(cameraObject.getPose().getY(), 3));
          widget.targetIdEntry.setDouble(cameraObject.getTarget().getDetectedObjectClassID());
          widget.confidenceEntry.setDouble(truncate(cameraObject.getConfidence(), 3));
        } else {
          widget.distanceEntry.setDouble(0.0);
          widget.poseXEntry.setDouble(0.0);
          widget.poseYEntry.setDouble(0.0);
          widget.targetIdEntry.setDouble(-1);
        }
      }
    }
  }

  /**
   * Truncates a double value to a specified number of decimal places.
   *
   * @param value The value to truncate.
   * @param places The number of decimal places to keep.
   * @return The truncated value.
   */
  private double truncate(double value, int places) {
    double scale = Math.pow(10, places);
    return Math.round(value * scale) / scale;
  }
}
