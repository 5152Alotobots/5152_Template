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
package frc.alotobots.library.subsystems.swervedrive.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.OI;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;

public class DefaultDrive extends Command {
  private final SwerveDriveSubsystem swerveDriveSubsystem;

  public DefaultDrive(SwerveDriveSubsystem swerveDriveSubsystem) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    addRequirements(swerveDriveSubsystem);
  }

  @Override
  public void execute() {
    // Get inputs from OI (already field-relative)
    Translation2d linearVelocity = OI.getDriverLinearVelocity(swerveDriveSubsystem);
    double omega = OI.getDriverRotation();

    // Convert to chassis speeds
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * swerveDriveSubsystem.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * swerveDriveSubsystem.getMaxLinearSpeedMetersPerSec(),
            omega * swerveDriveSubsystem.getMaxAngularSpeedRadPerSec());

    // Apply speeds to drive
    swerveDriveSubsystem.runVelocity(speeds);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
