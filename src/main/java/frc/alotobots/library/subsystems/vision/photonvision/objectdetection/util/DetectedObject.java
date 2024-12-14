//   ____  _ ____ ____       _    _        ___ _____ ___  ____     ___ _____ ____
//  | ___|/ | ___|___ \    / \  | |     / _ \_   _/ _ \| __) / _ \_   _/ ___|
//  |___ \| |___ \ __) | / _ \ | |    | | | || || | | |  _ \|| | || | \___ \
//   ___) | |___) / __/   / ___ \| |__  | |_| || || |_| | |_) | |_| || |  ___) |
//  |____/|_|____/_____| /_/   \_\_____\___/ |_| \___/|____/ \___/ |_| |____/
//
//
// 2025 ALOTOBOTS FRC 5152
// Robot Code
package frc.alotobots.library.subsystems.vision.photonvision.objectdetection.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.constants.ObjectDetectionConstants;
import org.photonvision.PhotonUtils;

public record DetectedObject(
    SwerveDriveSubsystem drive,
    Pose3d pose,
    int classId,
    Transform3d robotToCamera,
    double confidence,
    double lastUpdateTime) {

  /** Creates a DetectedObject with initial confidence */
  public static DetectedObject create(
      SwerveDriveSubsystem drive, Pose3d pose, int classId, Transform3d robotToCamera) {
    return new DetectedObject(
        drive,
        pose,
        classId,
        robotToCamera,
        ObjectDetectionConstants.INITIAL_CONFIDENCE,
        Timer.getFPGATimestamp());
  }

  /** Creates a default DetectedObject */
  public static DetectedObject createDefault(SwerveDriveSubsystem drive) {
    if (drive == null) {
      throw new IllegalArgumentException("SwerveDriveSubsystem cannot be null");
    }
    return new DetectedObject(
        drive,
        new Pose3d(),
        -1,
        null,
        ObjectDetectionConstants.INITIAL_CONFIDENCE,
        Timer.getFPGATimestamp());
  }

  /** Creates a new DetectedObject from raw vision data */
  public static DetectedObject fromVisionData(
      double yaw,
      double pitch,
      int classId,
      Transform3d robotToCamera,
      SwerveDriveSubsystem drive) {
    // Validate input parameters
    if (robotToCamera == null) {
      throw new IllegalArgumentException("robotToCamera must not be null");
    }

    // Validate game element configuration
    if (ObjectDetectionConstants.GAME_ELEMENTS.length == 0) {
      throw new IllegalStateException("No game elements configured");
    }

    if (classId >= ObjectDetectionConstants.GAME_ELEMENTS.length) {
      throw new IllegalArgumentException("Invalid class ID: " + classId);
    }
    GameElement matchedElement = ObjectDetectionConstants.GAME_ELEMENTS[classId];

    // Calculate distance using the matched element's height
    double targetToCameraDistance =
        PhotonUtils.calculateDistanceToTargetMeters(
            robotToCamera.getZ(),
            matchedElement.height(),
            robotToCamera.getRotation().getY(),
            Units.degreesToRadians(pitch));

    // Get the 2D translation in camera space
    Translation2d targetToCamera2d =
        PhotonUtils.estimateCameraToTargetTranslation(
            targetToCameraDistance, Rotation2d.fromDegrees(-yaw));

    // Convert the 2D translation to a 3D transform
    Transform3d cameraToTarget =
        new Transform3d(
            new Translation3d(
                targetToCamera2d.getX(), targetToCamera2d.getY(), matchedElement.height()),
            new Rotation3d());

    // Now combine the transforms to get target in robot space
    Transform3d targetToRobot = robotToCamera.plus(cameraToTarget);

    // If we have drive data, convert to field space
    if (drive != null) {
      Pose3d robotPose =
          new Pose3d(
              drive.getPose().getX(),
              drive.getPose().getY(),
              0.0,
              new Rotation3d(0, 0, drive.getPose().getRotation().getRadians()));
      return create(drive, robotPose.transformBy(targetToRobot), classId, robotToCamera);
    }

    // If no drive data, return in robot space
    return create(drive, new Pose3d().transformBy(targetToRobot), classId, robotToCamera);
  }

  public GameElement getGameElement() {
    if (classId < 0 || classId >= ObjectDetectionConstants.GAME_ELEMENTS.length) {
      return null;
    }
    return ObjectDetectionConstants.GAME_ELEMENTS[classId];
  }

  public double getDistance() {
    if (robotToCamera == null) {
      return 0;
    }
    return PhotonUtils.getDistanceToPose(drive.getPose(), pose.toPose2d());
  }

  public Rotation2d getAngle() {
    if (drive == null) {
      return new Rotation2d();
    }
    Translation2d robotToTarget =
        pose.toPose2d().getTranslation().minus(drive.getPose().getTranslation());
    return new Rotation2d(robotToTarget.getX(), robotToTarget.getY()).plus(new Rotation2d(Math.PI));
  }

  public DetectedObject withUpdatedConfidence() {
    double currentTime = Timer.getFPGATimestamp();
    double timeDelta = currentTime - lastUpdateTime;
    double newConfidence =
        Math.max(
            ObjectDetectionConstants.MIN_CONFIDENCE,
            confidence - (ObjectDetectionConstants.CONFIDENCE_DECAY_RATE * timeDelta));
    return new DetectedObject(drive, pose, classId, robotToCamera, newConfidence, currentTime);
  }

  public boolean matchesPosition(DetectedObject other) {
    if (other == null || other.pose == null || this.pose == null) {
      return false;
    }
    double deltaX = this.pose.getX() - other.pose.getX();
    double deltaY = this.pose.getY() - other.pose.getY();
    double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    return distance <= ObjectDetectionConstants.POSITION_MATCH_TOLERANCE;
  }

  public boolean isStale() {
    return confidence <= ObjectDetectionConstants.MIN_CONFIDENCE;
  }

  public DetectedObject refresh() {
    return new DetectedObject(
        drive,
        pose,
        classId,
        robotToCamera,
        ObjectDetectionConstants.INITIAL_CONFIDENCE,
        Timer.getFPGATimestamp());
  }

  @Override
  public String toString() {
    return String.format("Target at (%.2f, %.2f, %.2f)", pose.getX(), pose.getY(), pose.getZ());
  }
}
