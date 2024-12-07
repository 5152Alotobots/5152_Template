package frc.alotobots.library.vision.photonvision.objectdetection;

import static frc.alotobots.library.vision.photonvision.objectdetection.PhotonVisionObjectDetectionSubsystemConstants.GAME_ELEMENTS;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDriveSubsystem;
import lombok.Getter;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Represents an object detected by the PhotonVision vision processing system. Tracks the object's
 * 3D pose, confidence level, and provides utilities for calculating relative positions and angles.
 * Objects are tracked with confidence decay over time to handle intermittent detection issues.
 */
public class DetectedObject {
  @Getter private final SwerveDriveSubsystem drive;
  @Getter private final Pose3d pose;
  @Getter private final PhotonTrackedTarget target;
  @Getter private final Transform3d robotToCamera;
  @Getter private double confidence;

  /**
   * Gets the GameElement type of this detected object.
   *
   * @return The GameElement representing this object's type, or null if target is null
   */
  public GameElement getGameElement() {
    if (target == null) {
      return null;
    }
    int classId = target.getDetectedObjectClassID();
    if (classId >= GAME_ELEMENTS.length) {
      throw new IllegalArgumentException("Invalid class ID: " + classId);
    }
    return GAME_ELEMENTS[classId];
  }

  private double lastUpdateTime;

  /**
   * Creates a new DetectedObject with default attributes. This constructor initializes a basic
   * object with default pose and confidence values. The drive subsystem is required for pose
   * calculations relative to the robot's position.
   *
   * @param drive The SwerveDriveSubsystem to use for pose calculations
   * @throws IllegalArgumentException if drive is null
   */
  public DetectedObject(SwerveDriveSubsystem drive) {
    if (drive == null) {
      throw new IllegalArgumentException("SwerveDriveSubsystem cannot be null");
    }
    this.drive = drive;
    this.pose = new Pose3d();
    this.target = null;
    this.robotToCamera = null;
    this.confidence = PhotonVisionObjectDetectionSubsystemConstants.INITIAL_CONFIDENCE;
    this.lastUpdateTime = Timer.getFPGATimestamp();
  }

  /**
   * Creates a new DetectedObject with a specific pose and tracking information. Initializes all
   * tracking parameters including confidence and timestamp.
   *
   * @param pose The 3D pose of the detected object in field coordinates
   * @param target The raw PhotonVision target data
   * @param robotToCamera The transform from robot center to camera position
   * @param drive The drive subsystem for position calculations
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
    this.confidence = PhotonVisionObjectDetectionSubsystemConstants.INITIAL_CONFIDENCE;
    this.lastUpdateTime = Timer.getFPGATimestamp();
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
    int classId = target.getDetectedObjectClassID();
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

    return PhotonUtils.getDistanceToPose(drive.getState().Pose, pose.toPose2d());
  }

  /**
   * Gets the field-relative angle from the robot to the object. (Rotational)
   *
   * @return The angle in radians, or 0 if the drive is not set.
   */
  public Rotation2d getAngle() {
    if (drive == null) {
      return new Rotation2d();
    }

    // Get the field-relative vector from robot to target
    Translation2d robotToTarget =
        pose.toPose2d().getTranslation().minus(drive.getState().Pose.getTranslation());

    // Calculate the angle of this vector in field space, add PI to point towards target
    return new Rotation2d(robotToTarget.getX(), robotToTarget.getY()).plus(new Rotation2d(Math.PI));
  }

  /** Updates the confidence value based on time decay. */
  public void updateConfidence() {
    double currentTime = Timer.getFPGATimestamp();
    double timeDelta = currentTime - lastUpdateTime;
    confidence =
        Math.max(
            PhotonVisionObjectDetectionSubsystemConstants.MIN_CONFIDENCE,
            confidence
                - (PhotonVisionObjectDetectionSubsystemConstants.CONFIDENCE_DECAY_RATE
                    * timeDelta));
    lastUpdateTime = currentTime;
  }

  /**
   * Checks if this object's position matches another object within tolerance.
   *
   * @param other The other DetectedObject to compare with
   * @return true if positions match within tolerance
   */
  public boolean matchesPosition(DetectedObject other) {
    if (other == null || other.pose == null || this.pose == null) {
      return false;
    }

    // Calculate Euclidean distance between the two positions
    double deltaX = this.pose.getX() - other.pose.getX();
    double deltaY = this.pose.getY() - other.pose.getY();
    double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

    // Return true if the distance is within the tolerance
    return distance <= PhotonVisionObjectDetectionSubsystemConstants.POSITION_MATCH_TOLERANCE;
  }

  /**
   * Checks if this object should be considered stale based on confidence.
   *
   * @return true if confidence is at or below minimum
   */
  public boolean isStale() {
    return confidence <= PhotonVisionObjectDetectionSubsystemConstants.MIN_CONFIDENCE;
  }

  /** Refreshes the object's confidence to initial value and updates timestamp. */
  public void refresh() {
    confidence = PhotonVisionObjectDetectionSubsystemConstants.INITIAL_CONFIDENCE;
    lastUpdateTime = Timer.getFPGATimestamp();
  }

  @Override
  public String toString() {
    return String.format("Target at (%.2f, %.2f, %.2f)", pose.getX(), pose.getY(), pose.getZ());
  }
}
