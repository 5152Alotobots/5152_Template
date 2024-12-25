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
package frc.alotobots;

import static frc.alotobots.OI.driveFacingBestObjectButton;
import static frc.alotobots.OI.pathfindToBestObjectButton;
import static frc.alotobots.library.subsystems.vision.photonvision.objectdetection.constants.ObjectDetectionConstants.NOTE;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.alotobots.library.subsystems.bling.BlingSubsystem;
import frc.alotobots.library.subsystems.bling.commands.NoAllianceWaiting;
import frc.alotobots.library.subsystems.bling.commands.SetToAllianceColor;
import frc.alotobots.library.subsystems.bling.io.BlingIO;
import frc.alotobots.library.subsystems.bling.io.BlingIOReal;
import frc.alotobots.library.subsystems.bling.io.BlingIOSim;
import frc.alotobots.library.subsystems.swervedrive.ModulePosition;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.library.subsystems.swervedrive.commands.DefaultDrive;
import frc.alotobots.library.subsystems.swervedrive.commands.FeedforwardCharacterization;
import frc.alotobots.library.subsystems.swervedrive.commands.WheelRadiusCharacterization;
import frc.alotobots.library.subsystems.swervedrive.io.*;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.AprilTagSubsystem;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.constants.AprilTagConstants;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.io.AprilTagIO;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.io.AprilTagIOPhotonVision;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.io.AprilTagIOPhotonVisionSim;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.ObjectDetectionSubsystem;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.commands.DriveFacingBestObject;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.commands.PathfindToBestObject;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.constants.ObjectDetectionConstants;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io.ObjectDetectionIO;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io.ObjectDetectionIOPhotonVision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * The main container class for the robot. This class is responsible for initializing all
 * subsystems, configuring button bindings, and setting up autonomous commands. It follows a
 * dependency injection pattern for hardware IO to support simulation and replay modes.
 */
public class RobotContainer {
  /** The swerve drive subsystem that handles robot movement. */
  private final SwerveDriveSubsystem swerveDriveSubsystem;

  /** The AprilTag vision subsystem for robot localization. */
  private final AprilTagSubsystem aprilTagSubsystem;

  /** The object detection subsystem for game piece tracking. */
  private final ObjectDetectionSubsystem objectDetectionSubsystem;

  /** The LED control subsystem for robot status indication. */
  private final BlingSubsystem blingSubsystem;

  /** Dashboard chooser for selecting autonomous routines. */
  private final LoggedDashboardChooser<Command> autoChooser;

  /**
   * Constructs the RobotContainer and initializes all robot subsystems and commands. Different IO
   * implementations are used based on whether the robot is running in real, simulation, or replay
   * mode.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot hardware initialization
        swerveDriveSubsystem =
            new SwerveDriveSubsystem(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(ModulePosition.FRONT_LEFT.index),
                new ModuleIOTalonFX(ModulePosition.FRONT_RIGHT.index),
                new ModuleIOTalonFX(ModulePosition.BACK_LEFT.index),
                new ModuleIOTalonFX(ModulePosition.BACK_RIGHT.index));
        aprilTagSubsystem =
            new AprilTagSubsystem(
                swerveDriveSubsystem::addVisionMeasurement,
                new AprilTagIOPhotonVision(AprilTagConstants.CAMERA_CONFIGS[0]),
                new AprilTagIOPhotonVision(AprilTagConstants.CAMERA_CONFIGS[1]));
        objectDetectionSubsystem =
            new ObjectDetectionSubsystem(
                swerveDriveSubsystem::getPose,
                new ObjectDetectionIOPhotonVision(ObjectDetectionConstants.CAMERA_CONFIGS[0]));
        blingSubsystem = new BlingSubsystem(new BlingIOReal());
        break;

      case SIM:
        // Simulation hardware initialization
        swerveDriveSubsystem =
            new SwerveDriveSubsystem(
                new GyroIO() {},
                new ModuleIOSim(ModulePosition.FRONT_LEFT.index),
                new ModuleIOSim(ModulePosition.FRONT_RIGHT.index),
                new ModuleIOSim(ModulePosition.BACK_LEFT.index),
                new ModuleIOSim(ModulePosition.BACK_RIGHT.index));
        aprilTagSubsystem =
            new AprilTagSubsystem(
                swerveDriveSubsystem::addVisionMeasurement,
                new AprilTagIOPhotonVisionSim(
                    AprilTagConstants.CAMERA_CONFIGS[0], swerveDriveSubsystem::getPose),
                new AprilTagIOPhotonVisionSim(
                    AprilTagConstants.CAMERA_CONFIGS[1], swerveDriveSubsystem::getPose));
        objectDetectionSubsystem =
            new ObjectDetectionSubsystem(swerveDriveSubsystem::getPose, new ObjectDetectionIO() {});
        blingSubsystem = new BlingSubsystem(new BlingIOSim());
        break;

      default:
        // Replay mode initialization
        swerveDriveSubsystem =
            new SwerveDriveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        aprilTagSubsystem =
            new AprilTagSubsystem(
                swerveDriveSubsystem::addVisionMeasurement,
                new AprilTagIO() {},
                new AprilTagIO() {});
        objectDetectionSubsystem =
            new ObjectDetectionSubsystem(swerveDriveSubsystem::getPose, new ObjectDetectionIO() {});
        blingSubsystem = new BlingSubsystem(new BlingIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        new WheelRadiusCharacterization(swerveDriveSubsystem));
    autoChooser.addOption(
        "Drive Simple FF Characterization", new FeedforwardCharacterization(swerveDriveSubsystem));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        swerveDriveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        swerveDriveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)",
        swerveDriveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)",
        swerveDriveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureDefaultCommands();
    configureLogicCommands();
  }

  /**
   * Configures the default commands for each subsystem. These commands run automatically when no
   * other commands are scheduled for a subsystem.
   */
  private void configureDefaultCommands() {
    swerveDriveSubsystem.setDefaultCommand(new DefaultDrive(swerveDriveSubsystem).getCommand());
    blingSubsystem.setDefaultCommand(
        new NoAllianceWaiting(blingSubsystem).andThen(new SetToAllianceColor(blingSubsystem)));
  }

  /** Configures commands that are triggered by button presses or other logic conditions. */
  private void configureLogicCommands() {
    driveFacingBestObjectButton.toggleOnTrue(
        new DriveFacingBestObject(objectDetectionSubsystem, swerveDriveSubsystem, NOTE));
    pathfindToBestObjectButton.onTrue(
        new PathfindToBestObject(objectDetectionSubsystem, swerveDriveSubsystem, NOTE));
  }

  /**
   * Returns the command to run in autonomous mode.
   *
   * @return The command selected in the autonomous chooser
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
