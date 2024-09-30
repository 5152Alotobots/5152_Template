package frc.alotobots.library.vision.photonvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Handles telemetry for the Photonvision subsystem. */
public class PhotonvisionTelemetry {
  private final ShuffleboardTab photonvisionTab;
  private final ShuffleboardLayout poseList;
  private final Field2d field;

  // Pose entries
  private final GenericEntry poseXEntry;
  private final GenericEntry poseYEntry;
  private final GenericEntry rotationEntry;

  /** Constructs a new PhotonvisionTelemetry object. */
  public PhotonvisionTelemetry() {
    this.photonvisionTab = Shuffleboard.getTab("Photonvision");
    this.field = new Field2d();
    this.poseList = initializePoseList();

    // Initialize pose entries
    this.poseXEntry = poseList.add("Pose X", 0.0).getEntry();
    this.poseYEntry = poseList.add("Pose Y", 0.0).getEntry();
    this.rotationEntry = poseList.add("Rotation", 0.0).getEntry();

    initializeField();
    initializeOtherWidgets();
  }

  /**
   * Initializes the pose list in Shuffleboard.
   *
   * @return The initialized ShuffleboardLayout for pose information.
   */
  private ShuffleboardLayout initializePoseList() {
    return photonvisionTab
        .getLayout("Pose", BuiltInLayouts.kList)
        .withSize(2, 2)
        .withPosition(0, 0)
        .withProperties(Map.of("Label position", "LEFT"));
  }

  /** Initializes the field widget in Shuffleboard. */
  private void initializeField() {
    photonvisionTab.add("Field", field).withPosition(2, 0).withSize(6, 4);
  }

  /** Initializes other widgets in Shuffleboard. */
  private void initializeOtherWidgets() {
    photonvisionTab
        .addBoolean(
            "Vision Pose Estimation Enabled",
            () -> PhotonvisionSubsystemConstants.USE_VISION_POSE_ESTIMATION)
        .withPosition(0, 3)
        .withSize(2, 1);
    photonvisionTab
        .addBoolean(
            "Only Use Pose Estimation in Teleop",
            () -> PhotonvisionSubsystemConstants.ONLY_USE_POSE_ESTIMATION_IN_TELEOP)
        .withPosition(0, 4)
        .withSize(2, 1);
  }

  /**
   * Updates the Shuffleboard with the latest telemetry data.
   *
   * @param estimatedPose The estimated pose from Photonvision.
   * @param detectedTags The list of detected AprilTags.
   */
  public void updateShuffleboard(
      Optional<Pose2d> estimatedPose, List<PhotonTrackedTarget> detectedTags) {
    estimatedPose.ifPresent(
        pose -> {
          // Update pose entries with truncated values
          poseXEntry.setDouble(truncate(pose.getX(), 3));
          poseYEntry.setDouble(truncate(pose.getY(), 3));
          rotationEntry.setDouble(truncate(pose.getRotation().getDegrees(), 3));

          // Update field widget
          field.setRobotPose(pose);

          // Draw tracer lines to detected tags
          drawTracerLines(pose, detectedTags);
        });

    if (estimatedPose.isEmpty()) {
      poseXEntry.setString("N/A");
      poseYEntry.setString("N/A");
      rotationEntry.setString("N/A");
    }
  }

  /**
   * Draws tracer lines from the robot pose to detected AprilTags.
   *
   * @param robotPose The current robot pose.
   * @param detectedTags The list of detected AprilTags.
   */
  private void drawTracerLines(Pose2d robotPose, List<PhotonTrackedTarget> detectedTags) {
    field.getObjects().clear(); // Clear previous lines

    for (PhotonTrackedTarget tag : detectedTags) {
      Optional<Pose3d> tagPoseOptional = PhotonvisionSubsystemConstants.aprilTagFieldLayout.getTagPose(tag.getFiducialId());
      if (tagPoseOptional.isPresent()) {
        Pose2d tagPose = tagPoseOptional.get().toPose2d();

        // Create a trajectory (line) from robot to tag with more than 8 points
        List<Trajectory.State> states = new ArrayList<>();
        for (int i = 0; i <= 8; i++) {
          double t = i / 8.0;
          double x = robotPose.getX() + (tagPose.getX() - robotPose.getX()) * t;
          double y = robotPose.getY() + (tagPose.getY() - robotPose.getY()) * t;
          double rotation = robotPose.getRotation().interpolate(tagPose.getRotation(), t).getRadians();
          states.add(new Trajectory.State(t, 0, 0, new Pose2d(x, y, robotPose.getRotation().interpolate(tagPose.getRotation(), t)), 0));
        }
        Trajectory line = new Trajectory(states);

        // Add the line to the field with a unique name
        String lineName = "Tag" + tag.getFiducialId() + "Line";
        field.getObject(lineName).setTrajectory(line);
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
