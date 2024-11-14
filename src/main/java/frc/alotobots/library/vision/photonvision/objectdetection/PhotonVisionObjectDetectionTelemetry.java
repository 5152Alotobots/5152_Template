package frc.alotobots.library.vision.photonvision.objectdetection;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.ArrayList;
import java.util.List;

public class PhotonVisionObjectDetectionTelemetry {
  private final ShuffleboardTab tab;
  private final List<GenericEntry> distanceEntries;
  private final List<GenericEntry> poseEntries;
  private final Field2d field;

  public PhotonVisionObjectDetectionTelemetry() {
    tab = Shuffleboard.getTab("Object Detection");
    distanceEntries = new ArrayList<>();
    poseEntries = new ArrayList<>();

    // Create entries for each camera
    for (int i = 0; i < PhotonVisionObjectDetectionSubsystemConstants.CAMERAS.length; i++) {
      distanceEntries.add(
          tab.add("Camera " + i + " Distance (m)", 0)
              .withPosition(i * 2, 0)
              .withSize(2, 1)
              .getEntry());

      poseEntries.add(
          tab.add("Camera " + i + " Pose", "No target")
              .withPosition(i * 2, 2)
              .withSize(2, 1)
              .getEntry());
    }

    // Initialize Field2d widget
    field = new Field2d();
    tab.add("Field", field).withPosition(0, 3).withSize(6, 4);
  }

  public void updateObjects(List<DetectedObject> objects) {
    // Ensure we have valid data
    if (objects == null || objects.isEmpty()) {
      System.out.println("No objects to update in telemetry");
      return;
    }
    System.out.println("Updating telemetry with " + objects.size() + " objects");

    // Update field visualization
    if (!objects.isEmpty() && objects.get(0) != null) {
      DetectedObject firstObject = objects.get(0);
      if (firstObject.getDrive() != null) {
        // Update robot pose
        field.setRobotPose(firstObject.getDrive().getState().Pose);

        // Update target poses
        for (DetectedObject obj : objects) {
          if (obj != null && obj.getPose() != null) {
            field.getObject("Target" + objects.indexOf(obj)).setPose(obj.getPose().toPose2d());
          }
        }
      }
    }

    // Update each camera's telemetry entries
    for (int i = 0; i < objects.size() && i < distanceEntries.size(); i++) {
      DetectedObject obj = objects.get(i);
      if (obj != null) {
        // Update distance
        double distance = obj.getDistance();
        distanceEntries.get(i).setDouble(distance);

        // Update pose string
        String poseStr = obj.toString();
        poseEntries.get(i).setString(poseStr);
      } else {
        // Clear values if no object detected
        distanceEntries.get(i).setDouble(0.0);
        poseEntries.get(i).setString("No target");
      }
    }
  }
}
