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
package frc.alotobots.library.subsystems.vision.oculus;

import static frc.alotobots.library.subsystems.vision.oculus.constants.OculusConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.vision.oculus.io.OculusIO;
import frc.alotobots.library.subsystems.vision.oculus.io.OculusIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * The OculusSubsystem manages communication and data handling for the Oculus Quest VR headset used
 * for robot navigation and positioning.
 */
public class OculusSubsystem extends SubsystemBase {
  /** The IO interface for communicating with the Oculus hardware. */
  private final OculusIO io;

  /** Logged inputs from the Oculus device. */
  private final OculusIOInputsAutoLogged inputs = new OculusIOInputsAutoLogged();

  /**
   * Creates a new OculusSubsystem.
   *
   * @param io The IO interface for communicating with the Oculus hardware
   */
  public OculusSubsystem(OculusIO io) {
    this.io = io;
    Logger.recordOutput("Oculus/status", "Initialized");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Oculus", inputs);

    var oculusPose = getOculusPose();
    var robotPose = oculusPose.transformBy(ROBOT_TO_OCULUS.inverse());
    Logger.recordOutput("Oculus/poses/headsetPose", oculusPose);
    Logger.recordOutput("Oculus/poses/robotPose", robotPose);
  }

  /**
   * Gets the current yaw (rotation) of the Oculus headset.
   *
   * @return The yaw as a Rotation2d
   */
  private Rotation2d getOculusYaw() {
    return Rotation2d.fromDegrees(-inputs.eulerAngles[1]);
  }

  /**
   * Gets the current position of the Oculus headset.
   *
   * @return The position as a Translation2d
   */
  private Translation2d getOculusPosition() {
    return new Translation2d(inputs.position[2], -inputs.position[0]);
  }

  /**
   * Gets the current pose (position and rotation) of the Oculus headset.
   *
   * @return The pose of the Oculus headset
   */
  public Pose2d getOculusPose() {
    return new Pose2d(getOculusPosition(), getOculusYaw());
  }

  /**
   * Gets the current pose of the robot based on the Oculus headset position.
   *
   * @return The calculated robot pose
   */
  public Pose2d getRobotPose() {
    return getOculusPose().transformBy(ROBOT_TO_OCULUS.inverse());
  }

  /**
   * Gets the current MISO (Master In Slave Out) value from the Oculus.
   *
   * @return The current MISO value
   */
  public int getMisoValue() {
    return inputs.misoValue;
  }

  /**
   * Sets the MOSI (Master Out Slave In) value for the Oculus.
   *
   * @param value The MOSI value to set
   */
  public void setMosi(int value) {
    io.setMosi(value);
  }

  /**
   * Sets a new reset pose for the Oculus tracking system.
   *
   * @param pose The target pose to reset to
   */
  public void setResetPose(Pose2d pose) {
    var targetPose = pose.plus(ROBOT_TO_OCULUS);
    io.setResetPose(targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees());
  }

  /**
   * Logs a status message for the Oculus subsystem.
   *
   * @param message The status message to log
   */
  public void logStatus(String message) {
    Logger.recordOutput("Oculus/status", message);
  }

  /**
   * Logs the ping time for communication with the Oculus.
   *
   * @param timestamp The timestamp to log
   */
  public void logPingTime(double timestamp) {
    Logger.recordOutput("Oculus/ping/sendTime", timestamp);
  }
}
