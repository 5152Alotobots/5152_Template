package frc.alotobots.library.vision.limelight;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.*;
import java.util.Map;

/** Handles telemetry for the Limelight subsystem. */
public class LimelightTelemetry {
  private final ShuffleboardTab limelightTab;
  private final ShuffleboardLayout detectedObjectsLayout;

  /**
   * Constructs a new LimelightTelemetry object.
   *
   * @param limelightSubsystem The Limelight subsystem to provide telemetry for.
   */
  public LimelightTelemetry(LimelightSubsystem limelightSubsystem) {
    this.limelightTab = Shuffleboard.getTab("Limelight");
    this.detectedObjectsLayout = initializeShuffleboard(limelightSubsystem);
  }

  /**
   * Initializes the Shuffleboard layout for Limelight telemetry.
   *
   * @param limelightSubsystem The Limelight subsystem to provide telemetry for.
   * @return The ShuffleboardLayout for detected objects.
   */
  private ShuffleboardLayout initializeShuffleboard(LimelightSubsystem limelightSubsystem) {
    ShuffleboardLayout detectedObjectsLayout =
        limelightTab
            .getLayout("Detected Objects", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 1, "Number of rows", 3));

    detectedObjectsLayout.addStringArray(
        "Highest Confidence",
        () -> limelightSubsystem.getDetectedObjectList().sortByConfidence().toArray());

    if (limelightSubsystem.getSubSysDrive() != null) {
      detectedObjectsLayout.addStringArray(
          "Closest To Robot",
          () ->
              limelightSubsystem
                  .getDetectedObjectList()
                  .sortByPose(limelightSubsystem.getSubSysDrive().getState().Pose)
                  .toArray());
    } else {
      detectedObjectsLayout.addString("Closest To Robot", () -> "Cannot get Pose from drive!");
    }

    limelightTab.addBoolean("Limelight Connected", this::isLimelightConnected);
    limelightTab.addBoolean(
        "Object Detected", () -> LimelightLib.getTV(LimelightSubsystemConstants.NN_LIMELIGHT));

    return detectedObjectsLayout;
  }

  /**
   * Checks if the Limelight is connected and responding.
   *
   * @return true if the Limelight is connected, false otherwise.
   */
  boolean isLimelightConnected() {
    return !NetworkTableInstance.getDefault()
        .getTable(LimelightSubsystemConstants.NN_LIMELIGHT)
        .getEntry("json")
        .getString("")
        .isEmpty();
  }

  /**
   * Updates the Shuffleboard with the latest telemetry data.
   *
   * @param limelightSubsystem The Limelight subsystem to update telemetry for.
   */
  public void updateShuffleboard(LimelightSubsystem limelightSubsystem) {
    // The widgets are updated automatically through the lambda functions provided during
    // initialization
    // Add any additional dynamic updates here if needed
  }
}
