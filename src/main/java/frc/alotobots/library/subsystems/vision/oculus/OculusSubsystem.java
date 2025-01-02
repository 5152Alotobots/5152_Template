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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.vision.oculus.io.OculusIO;
import frc.alotobots.library.subsystems.vision.oculus.io.OculusIOInputsAutoLogged;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/**
 * The OculusSubsystem manages communication and pose estimation with an Oculus Quest VR headset.
 * This subsystem provides robot positioning using the Quest's inside-out tracking system.
 */
public class OculusSubsystem extends SubsystemBase {
    /** Status code indicating the Oculus system is ready for commands */
    private static final int STATUS_READY = 0;
    /** Status code indicating a heading reset operation has completed */
    private static final int STATUS_HEADING_RESET_COMPLETE = 99;
    /** Status code indicating a pose reset operation has completed */
    private static final int STATUS_POSE_RESET_COMPLETE = 98;
    /** Status code indicating a ping response has been received */
    private static final int STATUS_PING_RESPONSE = 97;

    /** IO interface for communicating with the Oculus hardware */
    private final OculusIO io;
    /** Logged inputs from the Oculus system */
    private final OculusIOInputsAutoLogged inputs = new OculusIOInputsAutoLogged();

    /** Flag indicating if a pose reset operation is currently in progress */
    @Getter private boolean poseResetInProgress = false;
    /** Flag indicating if a heading reset operation is currently in progress */
    @Getter private boolean headingResetInProgress = false;
    /** Flag indicating if a ping operation is currently in progress */
    @Getter private boolean pingInProgress = false;
    /** Previous connection state for detecting status changes */
    @Getter private boolean wasConnected = false;

    /** Timestamp when the current reset operation started */
    private double resetStartTime = 0;
    /** Counter for the number of reset attempts made */
    private int currentResetAttempt = 0;
    /** Previous timestamp from the Oculus Quest */
    private double lastTimestamp = 0.0;
    /** Most recent timestamp from the Oculus Quest */
    private double currentTimestamp = 0.0;
    /** Target pose for the current reset operation */
    private Pose2d pendingResetPose = null;

    /**
     * Creates a new OculusSubsystem with the specified IO interface.
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
        Logger.recordOutput("Oculus/status/poses/headsetPose", oculusPose);
        var robotPose = oculusPose.transformBy(ROBOT_TO_OCULUS.inverse());
        Logger.recordOutput("Oculus/status/poses/robotPose", robotPose);

        handleResetTimeout();
        handleResetCompletion();
        handlePingResponse();
    }

    /**
     * Checks if any reset operations have timed out and handles them accordingly.
     */
    private void handleResetTimeout() {
        double currentTime = Timer.getTimestamp();
        if (currentTime - resetStartTime > RESET_TIMEOUT_SECONDS) {
            if (headingResetInProgress) handleReset(true);
            if (poseResetInProgress) handleReset(false);
        }
    }

    /**
     * Handles reset operation retries and failures.
     *
     * @param isHeadingReset True if handling a heading reset, false if handling a pose reset
     */
    private void handleReset(boolean isHeadingReset) {
        if (currentResetAttempt < MAX_RESET_ATTEMPTS) {
            String resetType = isHeadingReset ? "Heading" : "Pose";
            Logger.recordOutput(
                    "Oculus/status",
                    resetType + " Reset attempt " + (currentResetAttempt + 1) + " timed out, retrying...");
            currentResetAttempt++;
            resetStartTime = Timer.getTimestamp();

            io.setMosi(0); // Clear

            if (isHeadingReset) {
                io.setMosi(1); // Heading Reset
            } else {
                io.setResetPose(
                        pendingResetPose.getX(),
                        pendingResetPose.getY(),
                        pendingResetPose.getRotation().getDegrees());
                io.setMosi(2); // Pose Reset
            }
        } else {
            Logger.recordOutput(
                    "Oculus/status",
                    (isHeadingReset ? "Heading" : "Pose")
                            + " Reset failed after "
                            + MAX_RESET_ATTEMPTS
                            + " attempts");
            if (isHeadingReset) {
                clearHeadingResetState();
            } else {
                clearPoseResetState();
            }
        }
    }

    /**
     * Checks for and handles completion of reset operations.
     */
    private void handleResetCompletion() {
        if (headingResetInProgress && inputs.misoValue == STATUS_HEADING_RESET_COMPLETE) {
            Logger.recordOutput(
                    "Oculus/status",
                    "Heading Reset completed successfully on attempt " + (currentResetAttempt + 1));
            clearHeadingResetState();
        }
        if (poseResetInProgress && inputs.misoValue == STATUS_POSE_RESET_COMPLETE) {
            Logger.recordOutput(
                    "Oculus/status",
                    "Pose Reset completed successfully on attempt " + (currentResetAttempt + 1));
            clearPoseResetState();
        }
    }

    /**
     * Handles ping response from the Oculus system.
     */
    private void handlePingResponse() {
        if (inputs.misoValue == STATUS_PING_RESPONSE) {
            Logger.recordOutput("Oculus/status", "Ping response received");
            io.setMosi(0); // Clear
            pingInProgress = false;
        }
    }

    /**
     * Clears the state associated with a pose reset operation.
     */
    private void clearPoseResetState() {
        Logger.recordOutput("Oculus/status", "Clearing pose reset state");
        poseResetInProgress = false;
        pendingResetPose = null;
        currentResetAttempt = 0;
        io.setMosi(0); // Clear
    }

    /**
     * Clears the state associated with a heading reset operation.
     */
    private void clearHeadingResetState() {
        Logger.recordOutput("Oculus/status", "Clearing heading reset state");
        headingResetInProgress = false;
        currentResetAttempt = 0;
        io.setMosi(0); // Clear
    }

    /**
     * Gets the current yaw (rotation) of the Oculus headset.
     *
     * @return The headset's yaw as a Rotation2d
     */
    private Rotation2d getOculusYaw() {
        return Rotation2d.fromDegrees(-inputs.eulerAngles[1]);
    }

    /**
     * Gets the current position of the Oculus headset.
     *
     * @return The headset's position as a Translation2d
     */
    private Translation2d getOculusPosition() {
        return new Translation2d(inputs.position[2], -inputs.position[0]);
    }

    /**
     * Gets the current pose (position and rotation) of the Oculus headset.
     *
     * @return The headset's pose
     */
    public Pose2d getOculusPose() {
        return new Pose2d(getOculusPosition(), getOculusYaw());
    }

    /**
     * Gets the current pose of the robot, derived from the Oculus headset's pose.
     *
     * @return The robot's pose
     */
    public Pose2d getRobotPose() {
        return getOculusPose().transformBy(ROBOT_TO_OCULUS.inverse());
    }

    /**
     * Initiates a pose reset operation to a specified target pose.
     *
     * @param targetPose The desired pose to reset to
     * @return True if the reset was initiated successfully, false otherwise
     */
    public boolean resetToPose(Pose2d targetPose) {
        if (poseResetInProgress) {
            Logger.recordOutput("Oculus/status", "Cannot reset pose - reset already in progress");
            return false;
        }

        if (inputs.misoValue != STATUS_READY) {
            Logger.recordOutput(
                    "Oculus/status", "Cannot reset pose - Quest busy (MISO=" + inputs.misoValue + ")");
            return false;
        }

        targetPose = targetPose.plus(ROBOT_TO_OCULUS);
        pendingResetPose = targetPose;
        Logger.recordOutput(
                "Oculus/status",
                String.format(
                        "Initiating pose reset to X:%.2f Y:%.2f Rot:%.2f°",
                        targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()));

        io.setResetPose(targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees());
        poseResetInProgress = true;
        resetStartTime = Timer.getTimestamp();
        currentResetAttempt = 0;
        io.setMosi(2); // Pose Reset

        return true;
    }

    /**
     * Initiates a heading reset operation to zero the current heading.
     *
     * @return True if the reset was initiated successfully, false otherwise
     */
    public boolean zeroHeading() {
        if (headingResetInProgress) {
            Logger.recordOutput("Oculus/status", "Cannot zero heading - reset already in progress");
            return false;
        }

        if (inputs.misoValue != STATUS_READY) {
            Logger.recordOutput(
                    "Oculus/status", "Cannot zero heading - Quest busy (MISO=" + inputs.misoValue + ")");
            return false;
        }

        Logger.recordOutput("Oculus/status", "Zeroing heading");
        headingResetInProgress = true;
        resetStartTime = Timer.getTimestamp();
        currentResetAttempt = 0;
        io.setMosi(1); // Heading Reset
        return true;
    }

    /**
     * Sends a ping command to the Oculus system to verify communication.
     *
     * @return True if the ping was initiated successfully, false otherwise
     */
    public boolean ping() {
        if (pingInProgress) {
            Logger.recordOutput("Oculus/status", "Cannot ping - ping already in progress");
            return false;
        }

        if (inputs.misoValue != STATUS_READY) {
            Logger.recordOutput(
                    "Oculus/status", "Cannot ping - system busy (MISO=" + inputs.misoValue + ")");
            return false;
        }

        Logger.recordOutput("Oculus/status", "Sending ping...");
        Logger.recordOutput("Oculus/ping/sendTime", Timer.getTimestamp());
        pingInProgress = true;
        io.setMosi(3); // Ping
        return true;
    }

    /**
     * Checks if the Oculus Quest is currently connected and sending valid data.
     * This is determined by monitoring if the timestamp values are increasing.
     *
     * @return true if the Quest is connected and sending data, false otherwise
     */
    public boolean isConnected() {
        double currentTime = Timer.getTimestamp();

        // Check if timestamps are increasing and within timeout window
        boolean timestampValid = currentTimestamp > lastTimestamp;
        boolean timeoutValid = (currentTime - currentTimestamp) < CONNECTION_TIMEOUT;

        return timestampValid && timeoutValid;
    }
}