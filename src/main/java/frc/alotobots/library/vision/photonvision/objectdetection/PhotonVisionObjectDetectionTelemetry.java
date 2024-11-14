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

  private static class CameraWidget {
    final ShuffleboardLayout layout;
    final GenericEntry connectionStatus;
    final GenericEntry enabledEntry;
    final GenericEntry distanceEntry;
    final GenericEntry poseXEntry;
    final GenericEntry poseYEntry;
    final GenericEntry targetIdEntry;

    CameraWidget(ShuffleboardTab tab, String cameraName, int position) {
      layout =
          tab.getLayout("Camera " + cameraName, BuiltInLayouts.kList)
              .withSize(2, 4)
              .withPosition(position * 2, 0)
              .withProperties(Map.of("Label position", "LEFT"));

      connectionStatus = layout.add("Connected", false).getEntry();
      enabledEntry =
          layout.add("Enabled", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
      distanceEntry = layout.add("Target Distance (m)", 0.0).getEntry();
      poseXEntry = layout.add("Target X", 0.0).getEntry();
      poseYEntry = layout.add("Target Y", 0.0).getEntry();
      targetIdEntry = layout.add("Target ID", -1).getEntry();
    }
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
  private void drawTracerLines(List<DetectedObject> objects) {
    // Clear any existing tracer lines
    field.getObject("tracerLines").setPoses();

    for (DetectedObject obj : objects) {
      if (obj != null && obj.getPose() != null) {
        int objIndex = objects.indexOf(obj);
        field.getObject("Target" + objIndex).setPose(obj.getPose().toPose2d());

        // Draw tracer line from robot to object
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
  }

  public PhotonVisionObjectDetectionTelemetry() {
    tab = Shuffleboard.getTab("Object Detection");

    // Initialize camera widgets
    for (int i = 0; i < PhotonVisionObjectDetectionSubsystemConstants.CAMERAS.length; i++) {
      PhotonCamera camera = PhotonVisionObjectDetectionSubsystemConstants.CAMERAS[i];
      if (camera != null) {
        cameraWidgets.add(new CameraWidget(tab, camera.getName(), i));
      }
    }

    // Initialize Field2d widget
    field = new Field2d();
    tab.add("Field", field).withPosition(0, 4).withSize(6, 4);
  }

  public void updateObjects(List<DetectedObject> objects) {
    // Clear all existing objects and tracer lines from the field
    field.getObject("tracerLines").setPoses();
    for (int i = 0; i < PhotonVisionObjectDetectionSubsystemConstants.CAMERAS.length; i++) {
      field.getObject("Target" + i).setPose(new Pose2d());
      field.getObject("TracerLine" + i).setTrajectory(new Trajectory());
    }

    // Update field visualization if we have valid data
    if (!objects.isEmpty() && objects.get(0) != null && objects.get(0).getDrive() != null) {
      field.setRobotPose(objects.get(0).getDrive().getState().Pose);

      // Only draw tracer lines for objects from enabled cameras
      List<DetectedObject> enabledObjects = new ArrayList<>();
      for (DetectedObject obj : objects) {
        // Find which camera this object came from
        for (int i = 0; i < PhotonVisionObjectDetectionSubsystemConstants.CAMERAS.length; i++) {
          if (PhotonVisionObjectDetectionSubsystemConstants.CAMERAS[i] != null
              && i < cameraWidgets.size()
              && cameraWidgets.get(i).enabledEntry.getBoolean(true)) {
            enabledObjects.add(obj);
            break;
          }
        }
      }

      // Update target poses and draw tracer lines only for enabled cameras
      drawTracerLines(enabledObjects);
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
          widget.targetIdEntry.setDouble(cameraObject.getTarget().getFiducialId());
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
