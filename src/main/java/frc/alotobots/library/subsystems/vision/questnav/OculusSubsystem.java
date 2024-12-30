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

    // Log all raw input values
    Logger.recordOutput("Oculus/debug/raw/eulerAngles", inputs.eulerAngles);
    Logger.recordOutput("Oculus/debug/raw/position", inputs.position);
    Logger.recordOutput("Oculus/debug/raw/misoValue", inputs.misoValue);
    Logger.recordOutput("Oculus/debug/state/poseResetInProgress", poseResetInProgress);
    Logger.recordOutput("Oculus/debug/state/headingResetInProgress", headingResetInProgress);
    Logger.recordOutput("Oculus/debug/state/currentResetAttempt", currentResetAttempt);
    Logger.recordOutput("Oculus/debug/state/resetStartTime", resetStartTime);

    // Log timing information
    double currentTime = Timer.getTimestamp();
    Logger.recordOutput("Oculus/debug/timing/currentTime", currentTime);
    Logger.recordOutput("Oculus/debug/timing/timeSinceReset", currentTime - resetStartTime);
    Logger.recordOutput(
        "Oculus/debug/timing/timeoutStatus",
        (currentTime - resetStartTime) > RESET_TIMEOUT_SECONDS);

    // Handle reset timeout
    if (currentTime - resetStartTime > RESET_TIMEOUT_SECONDS) {
      if (headingResetInProgress) {
        Logger.recordOutput("Oculus/debug/timeout", "Heading reset timeout occurred");
        handleResetTimeoutHeading();
      }
      if (poseResetInProgress) {
        Logger.recordOutput("Oculus/debug/timeout", "Pose reset timeout occurred");
        handleResetTimeoutPose();
      }
    }

    // Log reset completion status
    Logger.recordOutput(
        "Oculus/debug/resetCompletion/headingReset",
        headingResetInProgress && inputs.misoValue == 99);
    Logger.recordOutput(
        "Oculus/debug/resetCompletion/poseReset", poseResetInProgress && inputs.misoValue == 98);

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
      Logger.recordOutput("Oculus/debug/ping/responseTime", Timer.getTimestamp());
      io.setMosi(0);
    }

    // Log all pose calculations
    Translation2d oculusPos = getOculusPosition();
    Logger.recordOutput("Oculus/debug/calculations/oculusPosition/x", oculusPos.getX());
    Logger.recordOutput("Oculus/debug/calculations/oculusPosition/y", oculusPos.getY());

    double heading = getHeading();
    Logger.recordOutput("Oculus/debug/calculations/heading", heading);
    Logger.recordOutput("Oculus/debug/calculations/turnRate", getTurnRate());
    Logger.recordOutput("Oculus/debug/calculations/oculusYaw", getOculusYaw());

    // Log poses as objects
    Logger.recordOutput("Oculus/debug/poses/oculus", getOculusPose());
    Logger.recordOutput("Oculus/debug/poses/robot", getRobotPose());
    Logger.recordOutput("Oculus/debug/transforms/oculusToRobot", OCULUS_TO_ROBOT);

    // Original logging
    Logger.recordOutput("Oculus/debug/position", getOculusPosition());
    Logger.recordOutput("Oculus/Pose", getOculusPose());
    Logger.recordOutput("Oculus/RobotPose", getRobotPose());
  }

  private void handleResetTimeoutPose() {
    Logger.recordOutput("Oculus/debug/resetTimeout/pose/attemptNumber", currentResetAttempt);
    Logger.recordOutput("Oculus/debug/resetTimeout/pose/maxAttempts", MAX_RESET_ATTEMPTS);

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
    Logger.recordOutput("Oculus/debug/resetTimeout/heading/attemptNumber", currentResetAttempt);
    Logger.recordOutput("Oculus/debug/resetTimeout/heading/maxAttempts", MAX_RESET_ATTEMPTS);

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
    Logger.recordOutput("Oculus/debug/clearState", "Clearing pose reset state");
    poseResetInProgress = false;
    pendingResetPose = null;
    currentResetAttempt = 0;
    io.setMosi(0);
  }

  private void clearHeadingResetState() {
    Logger.recordOutput("Oculus/debug/clearState", "Clearing heading reset state");
    headingResetInProgress = false;
    currentResetAttempt = 0;
    io.setMosi(0);
  }

  public boolean resetToPose(Pose2d targetPose) {
    Logger.recordOutput("Oculus/debug/resetToPose/called", true);
    Logger.recordOutput("Oculus/debug/resetToPose/initial/x", targetPose.getX());
    Logger.recordOutput("Oculus/debug/resetToPose/initial/y", targetPose.getY());
    Logger.recordOutput(
        "Oculus/debug/resetToPose/initial/rotation", targetPose.getRotation().getDegrees());

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
    Logger.recordOutput("Oculus/debug/resetToPose/transformed/x", targetPose.getX());
    Logger.recordOutput("Oculus/debug/resetToPose/transformed/y", targetPose.getY());
    Logger.recordOutput(
        "Oculus/debug/resetToPose/transformed/rotation", targetPose.getRotation().getDegrees());

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
    Logger.recordOutput("Oculus/debug/zeroHeading/called", true);
    Logger.recordOutput("Oculus/debug/zeroHeading/currentMisoValue", inputs.misoValue);

    if (inputs.misoValue == 0) {
      Logger.recordOutput("Oculus/debug/status", "Zeroing heading");
      Logger.recordOutput("Oculus/debug/zeroHeading/previousYawOffset", yawOffset);
      yawOffset = inputs.eulerAngles[1];
      Logger.recordOutput("Oculus/debug/zeroHeading/newYawOffset", yawOffset);
      headingResetInProgress = true;
      io.setMosi(1);
      resetStartTime = Timer.getTimestamp();
    } else {
      Logger.recordOutput(
          "Oculus/debug/status",
          "Cannot zero heading - system busy (MISO=" + inputs.misoValue + ")");
    }
  }

  public double getHeading() {
    double heading = Rotation2d.fromDegrees(getOculusYaw()).getDegrees();
    Logger.recordOutput("Oculus/debug/getHeading/value", heading);
    return heading;
  }

  public double getTurnRate() {
    double turnRate = -getOculusYaw();
    Logger.recordOutput("Oculus/debug/getTurnRate/value", turnRate);
    return turnRate;
  }

  private float getOculusYaw() {
    float[] eulerAngles = inputs.eulerAngles;
    Logger.recordOutput("Oculus/debug/getOculusYaw/rawEulerAngle", eulerAngles[1]);
    Logger.recordOutput("Oculus/debug/getOculusYaw/yawOffset", yawOffset);

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
    var oculusPositionCompensated = getOculusPosition();
    Logger.recordOutput("Oculus/debug/poses/compensatedPosition", oculusPositionCompensated);

    var pose = new Pose2d(oculusPositionCompensated, Rotation2d.fromDegrees(getOculusYaw()));
    Logger.recordOutput("Oculus/debug/poses/rawPose", pose);

    return pose;
  }

  public Pose2d getRobotPose() {
    var oculusPose = getOculusPose();
    Logger.recordOutput("Oculus/debug/poses/currentOculusPose", oculusPose);

    var robotPose = getOculusPose().transformBy(OCULUS_TO_ROBOT);
    Logger.recordOutput("Oculus/debug/poses/finalRobotPose", robotPose);

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
