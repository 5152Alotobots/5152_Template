package frc.alotobots.library.vision.photonvision.objectdetection;

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
              .withSize(2, 5)
              .withPosition(position * 2, 0)
              .withProperties(Map.of("Label position", "LEFT"));

      connectionStatus = layout.add("Connected", false).getEntry();
      enabledEntry = layout.add("Enabled", true).getEntry();
      distanceEntry = layout.add("Target Distance (m)", 0.0).getEntry();
      poseXEntry = layout.add("Target X", 0.0).getEntry();
      poseYEntry = layout.add("Target Y", 0.0).getEntry();
      targetIdEntry = layout.add("Target ID", -1).getEntry();
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
    // Update field visualization if we have valid data
    if (!objects.isEmpty() && objects.get(0) != null && objects.get(0).getDrive() != null) {
      field.setRobotPose(objects.get(0).getDrive().getState().Pose);

      // Update target poses on field
      for (DetectedObject obj : objects) {
        if (obj != null && obj.getPose() != null) {
          field.getObject("Target" + objects.indexOf(obj)).setPose(obj.getPose().toPose2d());
        }
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
}
