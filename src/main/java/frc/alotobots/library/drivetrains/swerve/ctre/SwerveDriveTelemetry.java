package frc.alotobots.library.drivetrains.swerve.ctre;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.Map;

/** Handles telemetry for the Swerve Drive subsystem. */
public class SwerveDriveTelemetry {
  private final ShuffleboardTab driveTab;
  private final ShuffleboardLayout poseList;
  private final Field2d field;

  // Pose entries
  private final GenericEntry poseXEntry;
  private final GenericEntry poseYEntry;
  private final GenericEntry rotationEntry;

  /**
   * Constructs a new SwerveDriveTelemetry object.
   *
   * @param swerveDrive The SwerveDriveSubsystem to provide telemetry for.
   */
  public SwerveDriveTelemetry(SwerveDriveSubsystem swerveDrive) {
    this.driveTab = Shuffleboard.getTab("Drive");
    this.field = new Field2d();
    this.poseList = initializePoseList(swerveDrive);

    // Initialize pose entries
    this.poseXEntry = poseList.add("Pose X", 0.0).getEntry();
    this.poseYEntry = poseList.add("Pose Y", 0.0).getEntry();
    this.rotationEntry = poseList.add("Rotation", 0.0).getEntry();

    initializeField();
    initializeOtherWidgets(swerveDrive);
  }

  /**
   * Initializes the pose list in Shuffleboard.
   *
   * @param swerveDrive The SwerveDriveSubsystem to provide telemetry for.
   * @return The initialized ShuffleboardLayout for pose information.
   */
  private ShuffleboardLayout initializePoseList(SwerveDriveSubsystem swerveDrive) {
    return driveTab
        .getLayout("Pose", BuiltInLayouts.kList)
        .withSize(2, 3)
        .withPosition(0, 0)
        .withProperties(Map.of("Label position", "LEFT"));
  }

  /** Initializes the field widget in Shuffleboard. */
  private void initializeField() {
    driveTab.add("Field", field).withPosition(2, 0).withSize(6, 4);
  }

  /**
   * Initializes other widgets in Shuffleboard.
   *
   * @param swerveDrive The SwerveDriveSubsystem to provide telemetry for.
   */
  private void initializeOtherWidgets(SwerveDriveSubsystem swerveDrive) {
    driveTab
        .addBoolean(
            "Flip Pathplanner",
            () ->
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                    == DriverStation.Alliance.Red)
        .withPosition(0, 3)
        .withSize(2, 1);
  }

  /**
   * Updates the Shuffleboard with the latest telemetry data.
   *
   * @param swerveDrive The SwerveDriveSubsystem to update telemetry for.
   */
  public void updateShuffleboard(SwerveDriveSubsystem swerveDrive) {
    Pose2d currentPose = swerveDrive.getState().Pose;

    // Update pose entries with truncated values
    poseXEntry.setDouble(truncate(currentPose.getX(), 3));
    poseYEntry.setDouble(truncate(currentPose.getY(), 3));
    rotationEntry.setDouble(truncate(currentPose.getRotation().getDegrees(), 3));

    // Update field widget
    field.setRobotPose(currentPose);
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
