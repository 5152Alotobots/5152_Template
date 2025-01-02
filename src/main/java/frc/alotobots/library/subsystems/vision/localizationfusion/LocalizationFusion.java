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
package frc.alotobots.library.subsystems.vision.localizationfusion;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.vision.oculus.util.OculusPoseSource;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.util.AprilTagPoseSource;
import org.littletonrobotics.junction.Logger;

/**
 * A subsystem that fuses multiple pose sources for robot localization.
 *
 * <p>This system primarily uses the Meta Quest 3's SLAM capabilities for high-frequency (120Hz)
 * pose updates, with AprilTag detection serving as a validation and initialization mechanism. The
 * system follows a hierarchical approach:
 *
 * <p>1. Quest SLAM (Primary): Provides continuous, high-precision pose estimates 2. AprilTag System
 * (Secondary): Used for initial pose acquisition and ongoing validation 3. Emergency Odometry
 * (Fallback): Available when both primary systems fail
 *
 * <p>The system maintains a state machine to manage transitions between these sources based on
 * availability and reliability metrics.
 */
public class LocalizationFusion extends SubsystemBase implements StateTransitionLogger {
  /** Maximum acceptable difference between AprilTag and Quest poses for validation (meters) */
  private static final double APRILTAG_VALIDATION_THRESHOLD = 0.5;

  /** Update interval matching Quest's native 120Hz update rate (seconds) */
  private static final double POSE_UPDATE_INTERVAL = 1.0 / 120.0;

  /** Consumer interface for receiving pose updates */
  private final PoseVisionConsumer poseConsumer;

  /** Primary pose source using Quest SLAM */
  private final OculusPoseSource oculusSource;

  /** Secondary pose source using AprilTags */
  private final AprilTagPoseSource tagSource;

  /** State machine managing pose source transitions */
  private final LocalizationState state;

  /** Timestamp of last pose update */
  private double lastUpdateTime = 0.0;

  /** Timestamp of last valid Quest update */
  private double lastQuestUpdate = 0.0;

  /** Most recent validated pose from any source */
  private Pose2d lastValidatedPose = null;

  /** Previous DriverStation connection state */
  private boolean wasConnected = false;

  /** Functional interface for consuming pose updates from the fusion system. */
  @FunctionalInterface
  public interface PoseVisionConsumer {
    /**
     * Accepts a new pose update from the fusion system.
     *
     * @param pose The current robot pose in field coordinates
     * @param timestampSeconds The timestamp when this pose was measured
     * @param stdDevs Standard deviations for x, y, and rotation measurements
     */
    void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
  }

  /**
   * Creates a new LocalizationFusion subsystem.
   *
   * @param poseConsumer Consumer for receiving pose updates
   * @param oculusSource Primary pose source using Quest SLAM
   * @param aprilTagSource Secondary pose source using AprilTags
   */
  public LocalizationFusion(
      PoseVisionConsumer poseConsumer,
      OculusPoseSource oculusSource,
      AprilTagPoseSource aprilTagSource) {
    this.poseConsumer = poseConsumer;
    this.oculusSource = oculusSource;
    this.tagSource = aprilTagSource;
    this.state = new LocalizationState(this);
  }

  @Override
  public void periodic() {
    logSystemStatus();
    handleDriverStationConnection();
    updatePoses();
    updateState();
  }

  /** Logs current system status to NetworkTables for monitoring. */
  private void logSystemStatus() {
    Logger.recordOutput("PoseEstimator/State", state.getCurrentState().getDescription());
    Logger.recordOutput("PoseEstimator/OculusConnected", oculusSource.isConnected());
    Logger.recordOutput("PoseEstimator/AprilTagConnected", tagSource.isConnected());
  }

  /**
   * Handles DriverStation connection state changes. When reconnecting, attempts to reset to last
   * known good pose.
   */
  private void handleDriverStationConnection() {
    boolean isConnected = DriverStation.isDSAttached();
    if (!wasConnected && isConnected && lastValidatedPose != null) {
      Logger.recordOutput("PoseEstimator/Event", "DriverStation connected - initiating pose reset");
      if (resetToPose(lastValidatedPose)) {
        state.transitionTo(LocalizationState.State.RESETTING);
      }
    }
    wasConnected = isConnected;
  }

  /**
   * Updates pose estimates based on current state and available sources. Maintains rate limiting to
   * match Quest update frequency.
   */
  private void updatePoses() {
    double currentTime = Timer.getTimestamp();
    if (currentTime - lastUpdateTime < POSE_UPDATE_INTERVAL) {
      return;
    }
    lastUpdateTime = currentTime;

    switch (state.getCurrentState()) {
      case UNINITIALIZED:
        handleUninitializedState();
        break;
      case RESETTING:
        handleResettingState();
        break;
      case QUEST_PRIMARY:
        handleQuestPrimaryState(currentTime);
        break;
      case TAG_BACKUP:
        handleTagBackupState(currentTime);
        break;
      case EMERGENCY:
        Logger.recordOutput(
            "PoseEstimator/Event",
            "Failed to connect to any external pose source. Running with odometry only!");
        break;
    }
  }

  /** Handles system initialization using AprilTag poses. */
  private void handleUninitializedState() {
    Pose2d tagPose = tagSource.getCurrentPose();
    if (tagPose != null) {
      lastValidatedPose = tagPose;
      if (DriverStation.isDSAttached() && resetToPose(tagPose)) {
        state.transitionTo(LocalizationState.State.RESETTING);
        Logger.recordOutput("PoseEstimator/Event", "Received initial AprilTag pose - resetting");
      }
    }
  }

  /** Handles pose reset operations and transitions. */
  private void handleResettingState() {
    if (!oculusSource.isConnected()) {
      state.transitionTo(LocalizationState.State.TAG_BACKUP);
      Logger.recordOutput(
          "PoseEstimator/Event", "Quest unavailable during reset - using AprilTags");
      return;
    }
    if (!isResetInProgress()) {
      lastQuestUpdate = Timer.getTimestamp();
      state.transitionTo(LocalizationState.State.QUEST_PRIMARY);
      Logger.recordOutput("PoseEstimator/Event", "Pose reset complete - using Quest primary");
    }
  }

  /**
   * Handles primary Quest-based pose updates with AprilTag validation.
   *
   * @param currentTime Current system timestamp
   */
  private void handleQuestPrimaryState(double currentTime) {
    if (!oculusSource.isConnected()) {
      state.transitionTo(LocalizationState.State.TAG_BACKUP);
      Logger.recordOutput("PoseEstimator/Event", "Quest connection lost - switching to AprilTags");
      return;
    }

    Pose2d questPose = oculusSource.getCurrentPose();
    if (questPose != null) {
      if (validatePose(questPose)) {
        poseConsumer.accept(questPose, currentTime, oculusSource.getStdDevs());
        lastQuestUpdate = currentTime;
      } else {
        Logger.recordOutput("PoseEstimator/Event", "Quest pose validation failed");
        state.transitionTo(LocalizationState.State.TAG_BACKUP);
      }
    }
  }

  /**
   * Handles fallback AprilTag-based pose updates.
   *
   * @param currentTime Current system timestamp
   */
  private void handleTagBackupState(double currentTime) {
    if (oculusSource.isConnected()) {
      state.transitionTo(LocalizationState.State.QUEST_PRIMARY);
      Logger.recordOutput(
          "PoseEstimator/Event", "Quest connection restored - switching to primary");
      return;
    }

    Pose2d tagPose = tagSource.getCurrentPose();
    if (tagPose != null) {
      poseConsumer.accept(tagPose, currentTime, tagSource.getStdDevs());
      lastValidatedPose = tagPose;
    }
  }

  /**
   * Validates Quest pose against AprilTag measurements.
   *
   * @param pose Quest pose to validate
   * @return true if pose is valid or no validation possible
   */
  private boolean validatePose(Pose2d pose) {
    Pose2d tagPose = tagSource.getCurrentPose();
    if (tagPose == null) {
      return true; // No validation possible, assume valid
    }

    double poseError = tagPose.getTranslation().getDistance(pose.getTranslation());
    return poseError <= APRILTAG_VALIDATION_THRESHOLD;
  }

  /** Updates system state based on source availability. */
  private void updateState() {
    if (!state.isInState(
        LocalizationState.State.UNINITIALIZED, LocalizationState.State.RESETTING)) {
      if (oculusSource.isConnected()) {
        state.transitionTo(LocalizationState.State.QUEST_PRIMARY);
      } else {
        state.transitionTo(LocalizationState.State.TAG_BACKUP);
      }
    }
  }

  /**
   * Initiates a pose reset operation.
   *
   * @param pose Target pose to reset to
   * @return true if reset was initiated successfully
   */
  private boolean resetToPose(Pose2d pose) {
    return oculusSource.subsystem.resetToPose(pose);
  }

  /**
   * Checks if a pose reset operation is in progress.
   *
   * @return true if reset is ongoing
   */
  private boolean isResetInProgress() {
    return oculusSource.subsystem.isPoseResetInProgress();
  }

  /**
   * Manually triggers a pose reset using the most recent validated pose. This can be used as a
   * callback for manual reset requests.
   *
   * @return true if reset was initiated successfully, false otherwise
   */
  public boolean requestResetOculusPoseViaAprilTags() {
    Pose2d currentTagPose = tagSource.getCurrentPose();
    if (currentTagPose == null) {
      Logger.recordOutput(
          "PoseEstimator/Event", "Manual reset failed - no AprilTag pose available");
      return false;
    }

    Logger.recordOutput(
        "PoseEstimator/Event",
        "Manual reset requested using AprilTag pose - initiating pose reset");
    if (resetToPose(currentTagPose)) {
      state.transitionTo(LocalizationState.State.RESETTING);
      return true;
    }
    return false;
  }

  /**
   * Manually triggers a pose reset to a specific pose. Use this when you want to reset to a known
   * position rather than the last validated pose.
   *
   * @param targetPose The pose to reset to
   * @return true if reset was initiated successfully, false otherwise
   */
  public boolean requestResetOculusPose(Pose2d targetPose) {
    if (targetPose == null) {
      Logger.recordOutput("PoseEstimator/Event", "Manual reset failed - null pose provided");
      return false;
    }

    Logger.recordOutput(
        "PoseEstimator/Event", "Manual reset requested with custom pose - initiating pose reset");
    if (resetToPose(targetPose)) {
      state.transitionTo(LocalizationState.State.RESETTING);
      return true;
    }
    return false;
  }

  @Override
  public void logTransition(LocalizationState.State from, LocalizationState.State to) {
    Logger.recordOutput(
        "PoseEstimator/StateTransition",
        String.format("State transition: %s -> %s", from.name(), to.name()));
  }
}
