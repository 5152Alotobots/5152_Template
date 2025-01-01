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

import static frc.alotobots.library.subsystems.vision.questnav.constants.OculusConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.vision.questnav.OculusSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Command that resets the Oculus headset's position tracking to a specified pose. This is used to
 * align the Oculus coordinate system with the field coordinate system.
 */
public class ResetPoseCommand extends Command {
  /** Status code indicating the Oculus is ready for commands */
  private static final int STATUS_READY = 0;

  /** Status code indicating the pose reset is complete */
  private static final int STATUS_POSE_RESET_COMPLETE = 98;

  /** The Oculus subsystem instance */
  private final OculusSubsystem oculus;

  /** The target pose to reset to */
  private final Pose2d targetPose;

  /** Flag indicating if reset sequence has started */
  private boolean hasStarted;

  /** Counter for reset attempts */
  private int currentAttempt;

  /** Timestamp when current reset attempt started */
  private double startTime;

  /**
   * Creates a new ResetPoseCommand.
   *
   * @param oculus The Oculus subsystem to reset
   * @param targetPose The target pose to reset to
   */
  public ResetPoseCommand(OculusSubsystem oculus, Pose2d targetPose) {
    this.oculus = oculus;
    this.targetPose = targetPose;
    addRequirements(oculus);
  }

  @Override
  public void initialize() {
    hasStarted = false;
    currentAttempt = 0;
    startTime = 0;
    startReset();
  }

  /** Initiates a new pose reset attempt if the Oculus is ready. */
  private void startReset() {
    if (oculus.getMisoValue() == STATUS_READY) {
      oculus.setResetPose(targetPose);
      oculus.setMosi(2); // Pose Reset
      hasStarted = true;
      startTime = Timer.getTimestamp();
      currentAttempt++;
      Logger.recordOutput("Oculus/status", "Starting pose reset attempt " + currentAttempt);
    }
  }

  @Override
  public void execute() {
    if (!hasStarted) {
      startReset();
      return;
    }

    // Check for timeout
    if (Timer.getTimestamp() - startTime > RESET_TIMEOUT_SECONDS) {
      if (currentAttempt < MAX_RESET_ATTEMPTS) {
        Logger.recordOutput(
            "Oculus/status", "Pose reset attempt " + currentAttempt + " timed out, retrying...");
        oculus.setMosi(0); // Clear
        startReset();
      } else {
        Logger.recordOutput(
            "Oculus/status", "Pose reset failed after " + MAX_RESET_ATTEMPTS + " attempts");
        hasStarted = false;
      }
    }
  }

  @Override
  public boolean isFinished() {
    if (!hasStarted) return true;
    return oculus.getMisoValue() == STATUS_POSE_RESET_COMPLETE;
  }

  @Override
  public void end(boolean interrupted) {
    if (hasStarted) {
      oculus.setMosi(0); // Clear
      Logger.recordOutput(
          "Oculus/status",
          interrupted
              ? "Pose reset interrupted"
              : "Pose reset completed successfully on attempt " + currentAttempt);
    }
  }
}
