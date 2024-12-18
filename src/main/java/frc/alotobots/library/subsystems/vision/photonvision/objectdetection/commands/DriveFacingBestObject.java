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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.library.subsystems.swervedrive.commands.DefaultDrive;
import frc.alotobots.library.subsystems.swervedrive.commands.DriveFacingPose;
import frc.alotobots.library.subsystems.swervedrive.util.DriveCalculator;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.ObjectDetectionSubsystem;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io.ObjectDetectionIO;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.util.GameElement;

/**
 * Command that drives the robot while automatically facing the best detected object. Uses
 * PhotonVision object detection to identify targets and adjusts robot orientation accordingly.
 *
 * <p>This command: - Takes manual drive inputs for X/Y translation - Automatically rotates to face
 * the highest-confidence detected object - Falls back to manual rotation control when no objects
 * are detected - Allows temporary manual rotation override with a timeout
 *
 * <p>The command requires both the vision and drive subsystems to operate.
 */
public class DriveFacingBestObject extends Command {
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
   * Creates a new DriveFacingBestObject command.
   *
   * @param objectDetectionSubsystem The subsystem for detecting objects
   * @param swerveDriveSubsystem The subsystem for controlling robot movement
   */
  public DriveFacingBestObject(
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

  /**
   * Called repeatedly when this Command is scheduled to run. Controls robot movement while facing
   * detected objects.
   *
   * <p>The control flow: 1. If objects are detected: - Uses field-centric drive with automatic
   * rotation to face best object 2. If no objects detected: - Falls back to standard field-centric
   * drive with manual rotation 3. If manual rotation override is active: - Starts timeout timer for
   * returning to automatic facing
   */
  @Override
  public void execute() {
    var detectedObjects = objectDetectionSubsystem.getStableDetectedObjects();

    // Find first matching object based on priority order
    var matchingObject = java.util.Optional.<ObjectDetectionIO.DetectedObject>empty();
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
      Pose2d pose = objectDetectionSubsystem.toFieldRelative(matchingObject.get()).toPose2d();
      driveFacingPose.applyRequest(() -> pose); // Drive facing the pose
    } else {
      defaultDrive.applyRequest(); // Drive normal
    }

    // Rotation override timeout
    if (matchingObject.isPresent() && DriveCalculator.getDriverRotation() != 0) {
      overrideTimer.start();
    } else {
      overrideTimer.reset();
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
