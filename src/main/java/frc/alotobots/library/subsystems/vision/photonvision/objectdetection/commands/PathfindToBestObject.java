/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2024 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.alotobots.library.subsystems.vision.photonvision.objectdetection.commands;

import static frc.alotobots.library.subsystems.vision.photonvision.objectdetection.constants.ObjectDetectionConstants.GAME_ELEMENTS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.Constants;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.library.subsystems.swervedrive.commands.DefaultDrive;
import frc.alotobots.library.subsystems.swervedrive.commands.DriveFacingPose;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.ObjectDetectionSubsystem;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io.ObjectDetectionIO;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.util.GameElement;

public class PathfindToBestObject extends Command {
  /** The subsystem handling object detection via PhotonVision */
  private final ObjectDetectionSubsystem objectDetectionSubsystem;

  /** The swerve drive subsystem for robot movement */
  private final SwerveDriveSubsystem swerveDriveSubsystem;

  /** The names of game element types to target, in priority order */
  private final GameElement[] targetGameElementNames;

  private final DriveFacingPose driveFacingPose;
  private final DefaultDrive defaultDrive;

  Timer overrideTimer = new Timer();

  /**
   * Creates a new PathfindToBestObject command.
   *
   * @param objectDetectionSubsystem The subsystem for detecting objects
   * @param swerveDriveSubsystem The subsystem for controlling robot movement
   */
  public PathfindToBestObject(
      ObjectDetectionSubsystem objectDetectionSubsystem,
      SwerveDriveSubsystem swerveDriveSubsystem,
      GameElement... targetGameElementNames) {
    this.objectDetectionSubsystem = objectDetectionSubsystem;
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.targetGameElementNames = targetGameElementNames;
    this.driveFacingPose = new DriveFacingPose(swerveDriveSubsystem);
    this.defaultDrive = new DefaultDrive(swerveDriveSubsystem);

    addRequirements(swerveDriveSubsystem, objectDetectionSubsystem);
  }

  @Override
  public void execute() {
    var detectedObjects = objectDetectionSubsystem.getStableDetectedObjects();

    // Find first matching object based on priority order
    var matchingObject = java.util.Optional.<ObjectDetectionIO.DetectedObjectFieldRelative>empty();
    for (GameElement element : targetGameElementNames) {
      matchingObject =
          detectedObjects.stream()
              .filter(obj -> GAME_ELEMENTS[obj.classId()].equals(element))
              .findFirst();
      if (matchingObject.isPresent()) {
        break;
      }
    }

    if (matchingObject.isPresent()) {
      Pose2d robotPose = swerveDriveSubsystem.getPose();
      Pose2d objectPose = matchingObject.get().pose().toPose2d();

      // Calculate direction from robot to object
      Translation2d toObject = objectPose.getTranslation().minus(robotPose.getTranslation());
      double angle = Math.atan2(toObject.getY(), toObject.getX());

      // Determine which bumper dimension to use based on approach angle
      Distance offset =
          Math.abs(Math.cos(angle)) > Math.abs(Math.sin(angle))
              ? Constants.tunerConstants.getBumperLength()
              : Constants.tunerConstants.getBumperWidth();

      // Calculate target pose offset from object
      double offsetMeters = offset.in(Units.Meters);
      Translation2d offsetTranslation =
          new Translation2d(-Math.cos(angle) * offsetMeters, -Math.sin(angle) * offsetMeters);

      // Create target pose with offset
      Pose2d targetPose =
          new Pose2d(objectPose.getTranslation().plus(offsetTranslation), objectPose.getRotation());

      Command command =
          swerveDriveSubsystem.getPathFinderCommand(
              targetPose, LinearVelocity.ofBaseUnits(0, Units.MetersPerSecond));
      command.schedule();
    }
  }

  private static final double OVERRIDE_TIMEOUT_SECONDS = 0.1;

  /**
   * Returns true when the command should end. Ends when rotation override timeout has elapsed.
   *
   * @return true if the command should end
   */
  @Override
  public boolean isFinished() {
    return overrideTimer.hasElapsed(OVERRIDE_TIMEOUT_SECONDS);
  }
}
