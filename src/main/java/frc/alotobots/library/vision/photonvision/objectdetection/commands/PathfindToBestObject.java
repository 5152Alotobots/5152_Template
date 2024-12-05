package frc.alotobots.library.vision.photonvision.objectdetection.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDrivePathPlanner;
import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDriveSubsystem;
import frc.alotobots.library.drivetrains.swerve.ctre.mk4il22023.TunerConstants;
import frc.alotobots.library.vision.photonvision.objectdetection.PhotonVisionObjectDetectionSubsystem;

public class PathfindToBestObject extends InstantCommand {
  PhotonVisionObjectDetectionSubsystem objectDetectionSubsystem;
  SwerveDriveSubsystem swerveDriveSubsystem;
  SwerveDrivePathPlanner pathPlanner;

  public PathfindToBestObject(
      PhotonVisionObjectDetectionSubsystem objectDetectionSubsystem,
      SwerveDriveSubsystem swerveDriveSubsystem,
      SwerveDrivePathPlanner pathPlanner) {
    this.objectDetectionSubsystem = objectDetectionSubsystem;
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.pathPlanner = pathPlanner;

    addRequirements(swerveDriveSubsystem, objectDetectionSubsystem);
  }

  @Override
  public void initialize() {
    var detectedObjects = objectDetectionSubsystem.getDetectedObjects();
    if (detectedObjects.isEmpty()) {
      return;
    }

    Pose2d robotPose = swerveDriveSubsystem.getState().Pose;
    Pose2d objectPose = detectedObjects.get(0).getPose().toPose2d();

    // Calculate direction from robot to object
    Translation2d toObject = objectPose.getTranslation().minus(robotPose.getTranslation());
    double angle = Math.atan2(toObject.getY(), toObject.getX());

    // Determine which bumper dimension to use based on approach angle
    Distance offset =
        Math.abs(Math.cos(angle)) > Math.abs(Math.sin(angle))
            ? TunerConstants.BUMPER_LENGTH
            : TunerConstants.BUMPER_WIDTH;

    // Calculate target pose offset from object
    double offsetMeters = offset.in(Units.Meters);
    Translation2d offsetTranslation =
        new Translation2d(-Math.cos(angle) * offsetMeters, -Math.sin(angle) * offsetMeters);

    // Create target pose with offset
    Pose2d targetPose =
        new Pose2d(objectPose.getTranslation().plus(offsetTranslation), objectPose.getRotation());

    Command command =
        pathPlanner.getPathFinderCommand(
            targetPose, LinearVelocity.ofBaseUnits(0, Units.MetersPerSecond));
    command.schedule();
  }
}
