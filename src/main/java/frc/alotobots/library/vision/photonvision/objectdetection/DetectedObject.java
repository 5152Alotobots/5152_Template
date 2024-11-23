package frc.alotobots.library.vision.photonvision.objectdetection;

import static frc.alotobots.library.vision.photonvision.objectdetection.PhotonVisionObjectDetectionSubsystemConstants.GAME_ELEMENTS;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDriveSubsystem;
import lombok.Getter;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Represents an object detected by the PhotonVision system. */
public class DetectedObject {
  @Getter private final SwerveDriveSubsystem drive;
  @Getter private final Pose3d pose;
  @Getter private final PhotonTrackedTarget target;
  @Getter private final Transform3d robotToCamera;

  /**
   * Creates a new DetectedObject with default attributes.
   *
   * @param drive The SwerveDriveSubsystem to use for pose calculations
   */
  public DetectedObject(SwerveDriveSubsystem drive) {
    if (drive == null) {
      throw new IllegalArgumentException("SwerveDriveSubsystem cannot be null");
    }
    this.drive = drive;
    this.pose = new Pose3d();
    this.target = null;
    this.robotToCamera = null;
  }

  /**
   * Creates a new DetectedObject with a specific pose.
   *
   * @param pose The 3D pose of the detected object.
   */
  public DetectedObject(
      Pose3d pose,
      PhotonTrackedTarget target,
      Transform3d robotToCamera,
      SwerveDriveSubsystem drive) {
    this.pose = pose;
    this.target = target;
    this.robotToCamera = robotToCamera;
    this.drive = drive;
  }

  /**
   * Creates a new DetectedObject from a PhotonTrackedTarget.
   *
   * @param target The PhotonTrackedTarget from PhotonVision.
   * @param robotToCamera The transformation from the robot to the camera.
   * @return A new DetectedObject representing the tracked target.
   */
  public static DetectedObject fromPhotonTarget(
      PhotonTrackedTarget target, Transform3d robotToCamera, SwerveDriveSubsystem drive) {
    // Validate input parameters
    if (target == null || robotToCamera == null) {
      throw new IllegalArgumentException("Target and robotToCamera must not be null");
    }

    // Validate game element configuration
    if (GAME_ELEMENTS.length == 0) {
      throw new IllegalStateException("No game elements configured");
    }

    // Get game element from array (currently only Notes at index 0)
    int classId = 0; // Currently only supporting Notes
    // TODO: REPLACE THIS WITH target.getClassId(); once updated.
    if (classId >= GAME_ELEMENTS.length) {
      throw new IllegalArgumentException("Invalid class ID: " + classId);
    }
    GameElement matchedElement = GAME_ELEMENTS[classId];

    // Calculate distance using the matched element's height
    double targetToCameraDistance =
        PhotonUtils.calculateDistanceToTargetMeters(
            robotToCamera.getZ(),
            matchedElement.getHeight(),
            robotToCamera.getRotation().getY(),
            Units.degreesToRadians(target.getPitch()));

    // Get the 2D translation in camera space
    Translation2d targetToCamera2d =
        PhotonUtils.estimateCameraToTargetTranslation(
            targetToCameraDistance, Rotation2d.fromDegrees(-target.getYaw()));

    // Convert the 2D translation to a 3D transform
    Transform3d cameraToTarget =
        new Transform3d(
            new Translation3d(
                targetToCamera2d.getX(), targetToCamera2d.getY(), matchedElement.getHeight()),
            new Rotation3d() // If you need specific rotation, add it here
            );

    // Now combine the transforms to get target in robot space
    Transform3d targetToRobot = robotToCamera.plus(cameraToTarget);

    // If we have drive data, convert to field space
    if (drive != null) {
      Pose3d robotPose =
          new Pose3d(
              drive.getState().Pose.getX(),
              drive.getState().Pose.getY(),
              0.0,
              new Rotation3d(0, 0, drive.getState().Pose.getRotation().getRadians()));
      return new DetectedObject(robotPose.transformBy(targetToRobot), target, robotToCamera, drive);
    }

    // If no drive data, return in robot space
    return new DetectedObject(
        new Pose3d().transformBy(targetToRobot), target, robotToCamera, drive);
  }

  /**
   * Gets the distance from the center of the robot to the object.
   *
   * @return The distance in meters, or 0 if the drive is not set.
   */
  public double getDistance() {
    if (target == null || robotToCamera == null) {
      return 0;
    }

    return PhotonUtils.getDistanceToPose(drive.getPose(), pose.toPose2d());
  }

  /**
   * Gets the field-relative angle from the robot to the object. (Rotational)
   *
   * @return The angle in radians, or 0 if the drive is not set.
   */
  public double getAngle() {
    if (drive != null) {
      return Math.atan2(
          pose.getY() - drive.getState().Pose.getY(), pose.getX() - drive.getState().Pose.getX());
    }
    return 0;
  }

  @Override
  public String toString() {
    return String.format("Target at (%.2f, %.2f, %.2f)", pose.getX(), pose.getY(), pose.getZ());
  }
}