package frc.alotobots.library.vision.limelight;

import static frc.alotobots.library.vision.limelight.LimelightSubsystemConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.bling.BlingSubsystem;
import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDriveSubsystem;
import lombok.Getter;
import lombok.Setter;

/** Subsystem for interfacing with the Limelight vision system. */
public class LimelightSubsystem extends SubsystemBase {
  @Getter @Setter private BlingSubsystem subSysBling;
  @Getter @Setter private SwerveDriveSubsystem subSysDrive;
  @Getter private DetectedObjectList detectedObjectList;
  private LimelightTelemetry telemetry;

  /**
   * Constructs a new LimelightSubsystem.
   *
   * @param subSysBling The Bling subsystem for visual feedback.
   * @param subSysDrive The SwerveDrive subsystem for pose estimation.
   */
  public LimelightSubsystem(BlingSubsystem subSysBling, SwerveDriveSubsystem subSysDrive) {
    System.out.println("Initializing LimelightSubsystem");
    this.subSysBling = subSysBling;
    this.subSysDrive = subSysDrive;
    DetectedObject.setDrive(subSysDrive);
    this.detectedObjectList = new DetectedObjectList();
    this.telemetry = new LimelightTelemetry(this);
    System.out.println("LimelightSubsystem initialized");
  }

  /**
   * Calculates the distance to a target using the camera's parameters.
   *
   * @param targetHeight The height of the target from the floor in meters.
   * @param targetOffsetAngle_Vertical The vertical offset angle to the target in radians.
   * @return The distance to the target in meters.
   */
  public double targetDistanceMetersCamera(double targetHeight, double targetOffsetAngle_Vertical) {
    double angleToGoalRadians = LL_OFFSET.getRotation().getY() + targetOffsetAngle_Vertical;
    return (targetHeight - LL_OFFSET.getZ()) / Math.tan(angleToGoalRadians);
  }

  @Override
  public void periodic() {
    if (OBJECT_DETECTION_ENABLED && telemetry.isLimelightConnected()) {
      updateDetectedObjects();
    }

    detectedObjectList.update();
    telemetry.updateShuffleboard(this);
  }

  /** Updates the list of detected objects based on the latest Limelight data. */
  private void updateDetectedObjects() {
    boolean objectDetected = LimelightLib.getTV(NN_LIMELIGHT);
    if (objectDetected) {
      LimelightLib.LimelightResults latestResults = LimelightLib.getLatestResults(NN_LIMELIGHT);

      for (LimelightLib.LimelightTarget_Detector detection :
          latestResults.targetingResults.targets_Detector) {
        double horizontalOffset = Math.toRadians(detection.tx);
        double verticalOffset = Math.toRadians(-detection.ty); // Make CCW positive

        DetectedObject note =
            new DetectedObject(
                horizontalOffset, verticalOffset, DetectedObject.ObjectType.NOTE, LL_OFFSET);
        detectedObjectList.add(
            new DetectedObjectList.DetectedObjectPair(note, detection.confidence));
      }
    }
  }
}
