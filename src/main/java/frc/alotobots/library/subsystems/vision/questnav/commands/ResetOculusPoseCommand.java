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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.vision.questnav.OculusSubsystem;

public class ResetOculusPoseCommand extends Command {
  private final OculusSubsystem oculusSubsystem;
  private final Pose2d targetPose;
  private boolean resetInitiated = false;

  /**
   * Creates a new ResetOculusPoseCommand.
   *
   * @param oculusSubsystem The subsystem to reset
   * @param targetPose The pose to reset to
   */
  public ResetOculusPoseCommand(OculusSubsystem oculusSubsystem, Pose2d targetPose) {
    this.oculusSubsystem = oculusSubsystem;
    this.targetPose = targetPose;
    addRequirements(oculusSubsystem);
  }

  @Override
  public void initialize() {
    resetInitiated = oculusSubsystem.resetToPose(targetPose);
    if (!resetInitiated) {
      System.out.println("[ResetOculusPoseCommand] Failed to initiate pose reset");
    }
  }

  @Override
  public boolean isFinished() {
    // End command if:
    // 1. Reset failed to initiate
    // 2. Reset completed (no longer in progress)
    return !resetInitiated || !oculusSubsystem.isPoseResetInProgress();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("[ResetOculusPoseCommand] Reset interrupted or timed out");
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true; // Allow pose reset when robot disabled
  }
}
