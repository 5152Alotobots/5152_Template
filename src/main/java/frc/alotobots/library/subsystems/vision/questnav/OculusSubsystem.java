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
package frc.alotobots.library.subsystems.vision.questnav;

import static frc.alotobots.library.subsystems.vision.questnav.constants.OculusConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.vision.questnav.io.OculusIO;
import frc.alotobots.library.subsystems.vision.questnav.io.OculusIOInputsAutoLogged;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class OculusSubsystem extends SubsystemBase {
  private final OculusIO io;
  private final OculusIOInputsAutoLogged inputs = new OculusIOInputsAutoLogged();

  @Getter private boolean poseResetInProgress = false;
  @Getter private boolean headingResetInProgress = false;

  private double resetStartTime = 0;
  private int currentResetAttempt = 0;
  private Pose2d pendingResetPose = null;

  private float yawOffset = 0.0f;

  public OculusSubsystem(OculusIO io) {
    this.io = io;
    zeroHeading();
    Logger.recordOutput("Oculus/debug/status", "Initialized");
    Logger.recordOutput("Oculus/debug/yawOffset", yawOffset);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Oculus", inputs);

    getRobotPose();
    // Log timing information
    double currentTime = Timer.getTimestamp();

    // Handle reset timeout
    if (currentTime - resetStartTime > RESET_TIMEOUT_SECONDS) {
      if (headingResetInProgress) {
        handleResetTimeoutHeading();
      }
      if (poseResetInProgress) {
        handleResetTimeoutPose();
      }
    }

    // Handle reset completion
    if (headingResetInProgress && inputs.misoValue == 99) {
      Logger.recordOutput(
          "Oculus/debug/status",
          "Heading Reset completed successfully on attempt " + (currentResetAttempt + 1));
      clearHeadingResetState();
    }
    if (poseResetInProgress && inputs.misoValue == 98) {
      Logger.recordOutput(
          "Oculus/debug/status",
          "Pose Reset completed successfully on attempt " + (currentResetAttempt + 1));
      clearPoseResetState();
    }

    // Handle ping response
    if (inputs.misoValue == 97) {
      Logger.recordOutput("Oculus/debug/status", "Ping response received");
      io.setMosi(0);
    }
  }

  private void handleResetTimeoutPose() {
    if (currentResetAttempt < MAX_RESET_ATTEMPTS) {
      Logger.recordOutput(
          "Oculus/debug/status",
          "Pose Reset attempt " + (currentResetAttempt + 1) + " timed out, retrying...");
      currentResetAttempt++;
      resetStartTime = Timer.getTimestamp();

      // Log pending reset pose
      if (pendingResetPose != null) {
        Logger.recordOutput(
            "Oculus/debug/resetTimeout/pose/pendingReset/x", pendingResetPose.getX());
        Logger.recordOutput(
            "Oculus/debug/resetTimeout/pose/pendingReset/y", pendingResetPose.getY());
        Logger.recordOutput(
            "Oculus/debug/resetTimeout/pose/pendingReset/rotation",
            pendingResetPose.getRotation().getDegrees());
      }
      // Reset Mosi
      io.setMosi(0);
      // Retry the reset
      io.setResetPose(
          pendingResetPose.getX(),
          pendingResetPose.getY(),
          pendingResetPose.getRotation().getDegrees());
      io.setMosi(2);
    } else {
      Logger.recordOutput(
          "Oculus/debug/status", "Pose Reset failed after " + MAX_RESET_ATTEMPTS + " attempts");
      clearPoseResetState();
    }
  }

  private void handleResetTimeoutHeading() {
    if (currentResetAttempt < MAX_RESET_ATTEMPTS) {
      Logger.recordOutput(
          "Oculus/debug/status",
          "Heading Reset attempt " + (currentResetAttempt + 1) + " timed out, retrying...");
      currentResetAttempt++;
      resetStartTime = Timer.getTimestamp();

      // Reset Mosi
      io.setMosi(0);
      // Retry the reset
      io.setMosi(1);
    } else {
      Logger.recordOutput(
          "Oculus/debug/status", "Heading Reset failed after " + MAX_RESET_ATTEMPTS + " attempts");
      clearHeadingResetState();
    }
  }

  private void clearPoseResetState() {
    Logger.recordOutput("Oculus/debug/status", "Clearing pose reset state");
    poseResetInProgress = false;
    pendingResetPose = null;
    currentResetAttempt = 0;
    io.setMosi(0);
  }

  private void clearHeadingResetState() {
    Logger.recordOutput("Oculus/debug/status", "Clearing heading reset state");
    headingResetInProgress = false;
    currentResetAttempt = 0;
    io.setMosi(0);
  }

  public boolean resetToPose(Pose2d targetPose) {
    if (poseResetInProgress) {
      Logger.recordOutput("Oculus/debug/status", "Cannot reset pose - reset already in progress");
      return false;
    }

    if (inputs.misoValue != 0) {
      Logger.recordOutput(
          "Oculus/debug/status", "Cannot reset pose - Quest busy (MISO=" + inputs.misoValue + ")");
      return false;
    }

    targetPose = targetPose.plus(OCULUS_TO_ROBOT);
    pendingResetPose = targetPose;
    Logger.recordOutput(
        "Oculus/debug/status",
        String.format(
            "Initiating pose reset to X:%.2f Y:%.2f Rot:%.2f°",
            targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()));

    // Start reset process
    io.setResetPose(targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees());
    poseResetInProgress = true;
    resetStartTime = Timer.getTimestamp();
    currentResetAttempt = 0;
    io.setMosi(2);

    return true;
  }

  public void zeroHeading() {
    float[] eulerAngles = inputs.eulerAngles;
    yawOffset = eulerAngles[1];
    //    if (inputs.misoValue == 0) {
    //      Logger.recordOutput("Oculus/debug/status", "Zeroing heading");
    //      yawOffset = inputs.eulerAngles[1];
    //      headingResetInProgress = true;
    //      io.setMosi(1);
    //      resetStartTime = Timer.getTimestamp();
    //    } else {
    //      Logger.recordOutput(
    //          "Oculus/debug/status",
    //          "Cannot zero heading - system busy (MISO=" + inputs.misoValue + ")");
    //    }
  }

  public double getHeading() {
    double heading = Rotation2d.fromDegrees(getOculusYaw()).getDegrees();
    return heading;
  }

  public double getTurnRate() {
    return -getOculusYaw(); // Negative bc we are neg gyro
  }

  private float getOculusYaw() {
    float[] eulerAngles = inputs.eulerAngles;
    var ret = eulerAngles[1] - yawOffset;
    ret %= 360;
    if (ret < 0) {
      ret += 360;
    }
    Logger.recordOutput("Oculus/debug/getOculusYaw/calculatedValue", ret);
    return ret;
  }

  private Translation2d getOculusPosition() {
    Logger.recordOutput("Oculus/debug/getOculusPosition/rawZ", inputs.position[2]);
    Logger.recordOutput("Oculus/debug/getOculusPosition/rawX", inputs.position[0]);
    return new Translation2d(inputs.position[2], -inputs.position[0]);
  }

  public Pose2d getOculusPose() {
    var pose = new Pose2d(getOculusPosition(), Rotation2d.fromDegrees(getOculusYaw()));
    Logger.recordOutput("Oculus/debug/poses/headsetPose", pose);
    return pose;
  }

  public Pose2d getRobotPose() {
    var robotPose =
        new Pose2d(
            getOculusPose().getTranslation().minus(OCULUS_TO_ROBOT.getTranslation()),
            Rotation2d.fromDegrees(getOculusYaw()));
    Logger.recordOutput("Oculus/debug/poses/robotPose", robotPose);
    return robotPose;
  }

  public boolean ping() {
    Logger.recordOutput("Oculus/debug/ping/attempt", true);
    Logger.recordOutput("Oculus/debug/ping/currentMisoValue", inputs.misoValue);

    if (inputs.misoValue == 0) {
      Logger.recordOutput("Oculus/debug/status", "Sending ping...");
      Logger.recordOutput("Oculus/debug/ping/sendTime", Timer.getTimestamp());
      io.setMosi(3);
      return true;
    } else {
      Logger.recordOutput(
          "Oculus/debug/status", "Cannot ping - system busy (MISO=" + inputs.misoValue + ")");
      return false;
    }
  }
}
