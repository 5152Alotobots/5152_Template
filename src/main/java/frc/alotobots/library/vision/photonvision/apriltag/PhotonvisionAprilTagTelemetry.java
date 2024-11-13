package frc.alotobots.library.vision.photonvision.apriltag;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Handles telemetry for the PhotonVision AprilTag detection subsystem. */
public class PhotonvisionAprilTagTelemetry {
  private final ShuffleboardTab aprilTagTab;
  private final ShuffleboardLayout mainPoseList;
  private final Field2d field;

  // Main pose entries
  private final GenericEntry poseXEntry;
  private final GenericEntry poseYEntry;
  private final GenericEntry rotationEntry;

  // Per-camera widgets
  private final List<CameraWidget> cameraWidgets = new ArrayList<>();

  private static class CameraWidget {
    final ShuffleboardLayout poseList;
    final GenericEntry poseXEntry;
    final GenericEntry poseYEntry;
    final GenericEntry rotationEntry;
    final GenericEntry connectionStatus;
    final GenericEntry enabledEntry;

    // Track last known good values
    private String lastPoseX = "N/A";
    private String lastPoseY = "N/A";
    private String lastRotation = "N/A";

    CameraWidget(ShuffleboardTab tab, String cameraName, int position) {
      poseList =
          tab.getLayout("Camera " + cameraName, BuiltInLayouts.kList)
              .withSize(2, 3)
              .withPosition(position, 4)
              .withProperties(Map.of("Label position", "LEFT"));

      poseXEntry = poseList.add("Pose X", "N/A").getEntry();
      poseYEntry = poseList.add("Pose Y", "N/A").getEntry();
      rotationEntry = poseList.add("Rotation", "N/A").getEntry();
      connectionStatus = poseList.add("Connected", false).getEntry();
      enabledEntry = poseList.add("Enabled", true).getEntry();
    }
  }

  /** Constructs a new PhotonvisionTelemetry object. */
  public PhotonvisionAprilTagTelemetry() {
    this.aprilTagTab = Shuffleboard.getTab("AprilTag Vision");
    this.field = new Field2d();
    this.mainPoseList = initializeMainPoseList();

    // Initialize main pose entries
    this.poseXEntry = mainPoseList.add("Pose X", 0.0).getEntry();
    this.poseYEntry = mainPoseList.add("Pose Y", 0.0).getEntry();
    this.rotationEntry = mainPoseList.add("Rotation", 0.0).getEntry();

    initializeField();
    initializeOtherWidgets();
    initializeCameraWidgets();
  }

  /**
   * Initializes the pose list in Shuffleboard.
   *
   * @return The initialized ShuffleboardLayout for pose information.
   */
  private ShuffleboardLayout initializeMainPoseList() {
    return aprilTagTab
        .getLayout("Pose", BuiltInLayouts.kList)
        .withSize(2, 2)
        .withPosition(0, 0)
        .withProperties(Map.of("Label position", "LEFT"));
  }

  /** Initializes the field widget in Shuffleboard. */
  /**
   * Initializes the field widget in Shuffleboard. Creates and configures a Field2d widget to
   * display robot position and AprilTag locations.
   */
  private void initializeField() {
    aprilTagTab.add("Field", field).withPosition(2, 0).withSize(6, 4);
  }

  /** Initializes camera-specific widgets in Shuffleboard. */
  /**
   * Initializes camera-specific widgets in Shuffleboard. Creates a widget for each camera in the
   * CAMERAS array, displaying its pose and status information.
   */
  private void initializeCameraWidgets() {
    for (int i = 0; i < PhotonvisionAprilTagSubsystemConstants.CAMERAS.length; i++) {
      PhotonCamera camera = PhotonvisionAprilTagSubsystemConstants.CAMERAS[i];
      if (camera != null) {
        cameraWidgets.add(new CameraWidget(aprilTagTab, camera.getName(), i * 2));
      }
    }
  }

  /** Initializes other widgets in Shuffleboard. */
  private void initializeOtherWidgets() {
    aprilTagTab
        .addBoolean(
            "Vision Pose Estimation Enabled",
            () -> PhotonvisionAprilTagSubsystemConstants.USE_VISION_POSE_ESTIMATION)
        .withPosition(0, 2)
        .withSize(2, 1);
    aprilTagTab
        .addBoolean(
            "Only Use Pose Estimation in Teleop",
            () -> PhotonvisionAprilTagSubsystemConstants.ONLY_USE_POSE_ESTIMATION_IN_TELEOP)
        .withPosition(0, 3)
        .withSize(2, 1);
  }

  /**
   * Updates the camera-specific widgets with the latest data.
   *
   * @param cameras The array of PhotonCameras to get data from
   */
  /**
   * Updates the camera-specific widgets with the latest data from each camera. Displays connection
   * status, target detection results, and pose information.
   *
   * @param cameras Array of PhotonCameras to get data from
   */
  private void updateCameraWidgets(PhotonCamera[] cameras) {
    for (int i = 0; i < cameras.length && i < cameraWidgets.size(); i++) {
      PhotonCamera camera = cameras[i];
      CameraWidget widget = cameraWidgets.get(i);

      if (camera != null) {
        var result = camera.getLatestResult();
        boolean isConnected = camera.isConnected();

        widget.connectionStatus.setBoolean(isConnected);

        if (result.hasTargets()) {
          var bestTarget = result.getBestTarget();
          var camToTarget = bestTarget.getBestCameraToTarget();

          widget.poseXEntry.setDouble(truncate(camToTarget.getX(), 3));
          widget.poseYEntry.setDouble(truncate(camToTarget.getY(), 3));
          widget.rotationEntry.setDouble(truncate(camToTarget.getRotation().getZ(), 3));
        } else {
          widget.poseXEntry.setString("N/A");
          widget.poseYEntry.setString("N/A");
          widget.rotationEntry.setString("N/A");
        }
      }
    }
  }

  /**
   * Updates the Shuffleboard with the latest telemetry data.
   *
   * @param estimatedPose The estimated pose from Photonvision.
   * @param detectedTags The list of detected AprilTags.
   */
  public void updateShuffleboard(
      Optional<Pose2d> estimatedPose,
      List<PhotonTrackedTarget> detectedTags,
      List<edu.wpi.first.math.Pair<Integer, edu.wpi.first.math.Pair<Pose3d, Double>>>
          perCameraPoses) {
    // Update camera widgets
    updateCameraWidgets(PhotonvisionAprilTagSubsystemConstants.CAMERAS);

    // Update main pose display
    estimatedPose.ifPresent(
        pose -> {
          // Update pose entries with truncated values
          poseXEntry.setDouble(truncate(pose.getX(), 3));
          poseYEntry.setDouble(truncate(pose.getY(), 3));
          rotationEntry.setDouble(truncate(pose.getRotation().getDegrees(), 3));

          // Update field widget
          field.setRobotPose(pose);
        });

    if (estimatedPose.isEmpty()) {
      poseXEntry.setString("N/A");
      poseYEntry.setString("N/A");
      rotationEntry.setString("N/A");
    }

    // Create a set to track which cameras have updates
    boolean[] updatedCameras = new boolean[cameraWidgets.size()];

    // Draw tracer lines from each camera to its detected tags
    drawTracerLines(perCameraPoses);

    // Update per-camera poses
    for (edu.wpi.first.math.Pair<Integer, edu.wpi.first.math.Pair<Pose3d, Double>> cameraPose :
        perCameraPoses) {
      int cameraIndex = cameraPose.getFirst();
      if (cameraIndex < cameraWidgets.size()) {
        CameraWidget widget = cameraWidgets.get(cameraIndex);
        Pose3d pose = cameraPose.getSecond().getFirst();

        // Mark this camera as updated
        updatedCameras[cameraIndex] = true;

        // Update the widget entries with string values to ensure they display
        widget.poseXEntry.setString(String.valueOf(truncate(pose.getX(), 3)));
        widget.poseYEntry.setString(String.valueOf(truncate(pose.getY(), 3)));
        widget.rotationEntry.setString(String.valueOf(truncate(pose.getRotation().getZ(), 3)));

        // Draw individual camera poses on field
        String poseName = "Camera" + cameraIndex + "Pose";
        field.getObject(poseName).setPose(pose.toPose2d());
      }
    }

    // For cameras that got updates, store their new values as last known good
    for (int i = 0; i < cameraWidgets.size(); i++) {
      CameraWidget widget = cameraWidgets.get(i);
      if (updatedCameras[i]) {
        widget.lastPoseX = widget.poseXEntry.get().getString();
        widget.lastPoseY = widget.poseYEntry.get().getString();
        widget.lastRotation = widget.rotationEntry.get().getString();
      } else {
        // Use last known good values instead of N/A
        widget.poseXEntry.setString(widget.lastPoseX);
        widget.poseYEntry.setString(widget.lastPoseY);
        widget.rotationEntry.setString(widget.lastRotation);
      }
    }
  }

  /**
   * Draws tracer lines from each camera's detected pose to its detected AprilTags.
   *
   * @param cameraPoses List of camera poses and their indices
   */
  private void drawTracerLines(
      List<edu.wpi.first.math.Pair<Integer, edu.wpi.first.math.Pair<Pose3d, Double>>> cameraPoses) {
    // Clear previous lines
    field.getObject("tracerLines").setPoses();

    for (var cameraPose : cameraPoses) {
      int cameraIndex = cameraPose.getFirst();
      Pose2d cameraPose2d = cameraPose.getSecond().getFirst().toPose2d();
      PhotonCamera camera = PhotonvisionAprilTagSubsystemConstants.CAMERAS[cameraIndex];

      if (camera != null) {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
          for (PhotonTrackedTarget tag : result.getTargets()) {
            Optional<Pose3d> tagPoseOptional =
                PhotonvisionAprilTagSubsystemConstants.fieldLayout.getTagPose(tag.getFiducialId());

            if (tagPoseOptional.isPresent()) {
              Pose2d tagPose = tagPoseOptional.get().toPose2d();

              // Create a trajectory (line) from camera to tag
              List<Trajectory.State> states = new ArrayList<>();
              for (int i = 0; i <= 8; i++) {
                double t = i / 8.0;
                double x = cameraPose2d.getX() + (tagPose.getX() - cameraPose2d.getX()) * t;
                double y = cameraPose2d.getY() + (tagPose.getY() - cameraPose2d.getY()) * t;
                states.add(
                    new Trajectory.State(t, 0, 0, new Pose2d(x, y, cameraPose2d.getRotation()), 0));
              }
              Trajectory line = new Trajectory(states);

              // Add the line to the field with a unique name for this camera-tag pair
              String lineName = "Camera" + cameraIndex + "Tag" + tag.getFiducialId() + "Line";
              field.getObject(lineName).setTrajectory(line);

              // Draw a box for the tag position
              String boxName = "Tag" + tag.getFiducialId() + "Box";
              field.getObject(boxName).setPose(tagPose);
            }
          }
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
  /**
   * Truncates a double value to a specified number of decimal places. Used to format numeric values
   * for display in Shuffleboard widgets.
   *
   * @param value The value to truncate
   * @param places The number of decimal places to keep
   * @return The truncated value
   */
  private double truncate(double value, int places) {
    double scale = Math.pow(10, places);
    return Math.round(value * scale) / scale;
  }
}
