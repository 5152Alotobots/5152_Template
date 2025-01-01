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
package frc.alotobots.library.subsystems.vision.oculus.commands;

import static frc.alotobots.library.subsystems.vision.oculus.constants.OculusConstants.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.vision.oculus.OculusSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Command that sends a ping request to the Oculus headset and waits for a response. Used to verify
 * communication with the headset is working properly.
 */
public class PingCommand extends Command {
  /** Status code indicating the Oculus is ready for commands */
  private static final int STATUS_READY = 0;

  /** Status code indicating the Oculus has responded to ping */
  private static final int STATUS_PING_RESPONSE = 97;

  /** The Oculus subsystem instance */
  private final OculusSubsystem oculus;

  /** Flag indicating if ping sequence has started */
  private boolean hasStarted;

  /** Counter for ping attempts */
  private int currentAttempt;

  /** Timestamp when current ping attempt started */
  private double startTime;

  /**
   * Creates a new PingCommand.
   *
   * @param oculus The Oculus subsystem to ping
   */
  public PingCommand(OculusSubsystem oculus) {
    this.oculus = oculus;
    addRequirements(oculus);
  }

  @Override
  public void initialize() {
    startPing();
    hasStarted = false;
    currentAttempt = 0;
    startTime = 0;
  }

  /** Initiates a new ping attempt if the Oculus is ready. */
  private void startPing() {
    if (oculus.getMisoValue() == STATUS_READY) {
      oculus.logStatus("Starting ping attempt " + currentAttempt);
      oculus.logPingTime(Timer.getTimestamp());
      oculus.setMosi(3); // Ping
      hasStarted = true;
      startTime = Timer.getTimestamp();
      currentAttempt++;
    }
  }

  @Override
  public void execute() {
    if (!hasStarted) {
      startPing();
      return;
    }

    // Check for timeout
    if (Timer.getTimestamp() - startTime > RESET_TIMEOUT_SECONDS) {
      if (currentAttempt < MAX_RESET_ATTEMPTS) {
        Logger.recordOutput(
            "Oculus/status", "Ping attempt " + currentAttempt + " timed out, retrying...");
        oculus.setMosi(0); // Clear
        startPing();
      } else {
        Logger.recordOutput(
            "Oculus/status", "Ping failed after " + MAX_RESET_ATTEMPTS + " attempts");
        hasStarted = false;
      }
    }
  }

  @Override
  public boolean isFinished() {
    if (!hasStarted) return true;
    return oculus.getMisoValue() == STATUS_PING_RESPONSE;
  }

  @Override
  public void end(boolean interrupted) {
    if (hasStarted) {
      oculus.setMosi(0); // Clear
      Logger.recordOutput(
          "Oculus/status",
          interrupted
              ? "Ping interrupted"
              : "Ping completed successfully on attempt " + currentAttempt);
    }
  }
}
