package frc.alotobots.library.vision.photonvision.objectdetection.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDrivePathPlanner;
import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDriveSubsystem;
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
    Pose2d bestObjectPose =
        objectDetectionSubsystem.getDetectedObjects().get(0).getPose().toPose2d();
    Command command =
        pathPlanner.getPathFinderCommand(
            bestObjectPose, LinearVelocity.ofBaseUnits(0, Units.MetersPerSecond));
    command.schedule();
  }
}
