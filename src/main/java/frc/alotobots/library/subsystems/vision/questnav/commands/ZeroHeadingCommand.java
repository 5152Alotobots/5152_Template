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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.vision.questnav.OculusSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Command that zeros the heading (yaw) of the Oculus headset. This sets the current heading as the
 * zero reference point for rotation tracking.
 */
public class ZeroHeadingCommand extends Command {
  /** Status code indicating the Oculus is ready for commands */
  private static final int STATUS_READY = 0;

  /** Status code indicating the heading reset is complete */
  private static final int STATUS_HEADING_RESET_COMPLETE = 99;

  /** The Oculus subsystem instance */
  private final OculusSubsystem oculus;

  /** Flag indicating if reset sequence has started */
  private boolean hasStarted;

  /** Counter for reset attempts */
  private int currentAttempt;

  /** Timestamp when current reset attempt started */
  private double startTime;

  /**
   * Creates a new ZeroHeadingCommand.
   *
   * @param oculus The Oculus subsystem to reset
   */
  public ZeroHeadingCommand(OculusSubsystem oculus) {
    this.oculus = oculus;
    addRequirements(oculus);
  }

  @Override
  public void initialize() {
    startReset();
    hasStarted = false;
    currentAttempt = 0;
    startTime = 0;
  }

  /** Initiates a new heading reset attempt if the Oculus is ready. */
  private void startReset() {
    if (oculus.getMisoValue() == STATUS_READY) {
      oculus.setMosi(1); // Heading Reset
      hasStarted = true;
      startTime = Timer.getTimestamp();
      currentAttempt++;
      Logger.recordOutput("Oculus/status", "Starting heading reset attempt " + currentAttempt);
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
            "Oculus/status", "Heading reset attempt " + currentAttempt + " timed out, retrying...");
        oculus.setMosi(0); // Clear
        startReset();
      } else {
        Logger.recordOutput(
            "Oculus/status", "Heading reset failed after " + MAX_RESET_ATTEMPTS + " attempts");
        hasStarted = false;
      }
    }
  }

  @Override
  public boolean isFinished() {
    if (!hasStarted) return true;
    return oculus.getMisoValue() == STATUS_HEADING_RESET_COMPLETE;
  }

  @Override
  public void end(boolean interrupted) {
    if (hasStarted) {
      oculus.setMosi(0); // Clear
      Logger.recordOutput(
          "Oculus/status",
          interrupted
              ? "Heading reset interrupted"
              : "Heading reset completed successfully on attempt " + currentAttempt);
    }
  }
}
