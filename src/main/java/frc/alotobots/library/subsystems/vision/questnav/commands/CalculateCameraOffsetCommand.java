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
package frc.alotobots.library.subsystems.vision.questnav.commands;

import static frc.alotobots.library.subsystems.vision.questnav.constants.OculusConstants.OCULUS_TO_ROBOT;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.vision.questnav.OculusSubsystem;
import org.littletonrobotics.junction.Logger;

public class CalculateCameraOffsetCommand extends Command {
  private final OculusSubsystem oculusSubsystem;
  private Pose2d pose1;

  public CalculateCameraOffsetCommand(OculusSubsystem oculusSubsystem) {
    this.oculusSubsystem = oculusSubsystem;
    addRequirements(oculusSubsystem);
  }

  @Override
  public void initialize() {
    pose1 = oculusSubsystem.getOculusPose();
  }

  @Override
  public void end(boolean interrupted) {
    Pose2d pose2 = oculusSubsystem.getOculusPose();
    Transform2d offset = solveRobotToCamera(pose1, pose2, OCULUS_TO_ROBOT.getRotation());
    Logger.recordOutput("Oculus/debug/empiricalOffset", offset);
  }

  private static Transform2d solveRobotToCamera(
      Pose2d cameraPose1, Pose2d cameraPose2, Rotation2d angleOnRobot) {
    // Extract the camera positions and rotations
    double x1 = cameraPose1.getTranslation().getX();
    double y1 = cameraPose1.getTranslation().getY();
    double x2 = cameraPose2.getTranslation().getX();
    double y2 = cameraPose2.getTranslation().getY();
    double theta1 = cameraPose1.getRotation().getRadians();
    double theta2 = cameraPose2.getRotation().getRadians();

    // Compute the coefficients for x and y
    double cos1 = Math.cos(theta1);
    double sin1 = Math.sin(theta1);
    double cos2 = Math.cos(theta2);
    double sin2 = Math.sin(theta2);

    // Compute the determinant (denominator) for solving the system
    double denominator = (cos1 - cos2) * (cos1 - cos2) + (sin1 - sin2) * (sin1 - sin2);

    // Calculate x and y
    double x = ((x2 - x1) * (cos1 - cos2) + (y2 - y1) * (sin1 - sin2)) / -denominator;
    double y = ((x2 - x1) * (sin1 - sin2) + (y2 - y1) * (cos2 - cos1)) / denominator;

    // Return the robotToCamera transform as a Transform2d
    return new Transform2d(new Translation2d(x, y).rotateBy(angleOnRobot), angleOnRobot);
  }
}
