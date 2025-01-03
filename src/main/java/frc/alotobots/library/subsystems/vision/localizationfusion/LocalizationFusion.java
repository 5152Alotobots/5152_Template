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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.vision.oculus.util.OculusPoseSource;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.util.AprilTagPoseSource;
import org.littletonrobotics.junction.Logger;

public class LocalizationFusion extends SubsystemBase implements StateTransitionLogger {
  /** Maximum acceptable difference between AprilTag and Quest poses for validation (meters) */
  private static final double APRILTAG_VALIDATION_THRESHOLD = 1.0; // More lenient with Quest

  /** Stricter threshold for validating poses during initialization */
  private static final double INIT_VALIDATION_THRESHOLD = 0.3; // Stricter during init

  /** Update interval matching Quest's native 120Hz update rate (seconds) */
  private static final double POSE_UPDATE_INTERVAL = 1.0 / 120.0;

  /** Time window for considering a Quest disconnect as temporary (seconds) */
  private static final double QUICK_RECONNECT_WINDOW = 5.0;

  /** Time required for Quest initialization (seconds) */
  private static final double QUEST_INIT_TIMEOUT = 2.0;

  /** Time required for AprilTag initialization (seconds) */
  private static final double TAG_INIT_TIMEOUT = 1.0;

  /** Minimum valid Quest updates required for initialization */
  private static final int MIN_QUEST_VALID_UPDATES = 10;

  /** Minimum valid AprilTag updates required for initialization */
  private static final int MIN_TAG_VALID_UPDATES = 3;

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

  // Quest initialization tracking
  private double questInitStartTime = 0.0;
  private int consecutiveValidQuestUpdates = 0;
  private Pose2d lastQuestInitPose = null;
  private boolean questInitialized = false;
  private double lastQuestDisconnectTime = 0.0;
  private boolean hadPreviousCalibration = false;

  // AprilTag initialization tracking
  private double tagInitStartTime = 0.0;
  private int consecutiveValidTagUpdates = 0;
  private Pose2d lastTagInitPose = null;
  private boolean tagInitialized = false;

  /** Creates a new LocalizationFusion subsystem. */
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

    // Handle initialization of both sources
    if (!questInitialized && oculusSource.isConnected()) {
      handleQuestInitialization();
    }
    if (!tagInitialized && tagSource.isConnected()) {
      handleTagInitialization();
    }

    // Only proceed with normal updates if at least one source is initialized
    if (questInitialized || tagInitialized) {
      updatePoses();
      updateState();
    } else if (!state.isInState(LocalizationState.State.EMERGENCY)) {
      state.transitionTo(LocalizationState.State.EMERGENCY);
    }

    // Log initialization and status
    logDetailedStatus();
  }

  /**
   * Handles initialization of the Quest SLAM system. Differentiates between temporary disconnects
   * and full reboots.
   */
  private void handleQuestInitialization() {
    if (!oculusSource.isConnected()) {
      if (questInitialized) {
        lastQuestDisconnectTime = Timer.getTimestamp();
        hadPreviousCalibration = true;
      }
      resetQuestInitialization();
      return;
    }

    // Check if this is a quick reconnect with maintained calibration
    if (hadPreviousCalibration
        && (Timer.getTimestamp() - lastQuestDisconnectTime) < QUICK_RECONNECT_WINDOW) {

      // Verify the pose hasn't jumped dramatically
      Pose2d questPose = oculusSource.getCurrentPose();
      if (questPose != null && lastQuestInitPose != null) {
        double poseJump =
            questPose.getTranslation().getDistance(lastQuestInitPose.getTranslation());

        if (poseJump < APRILTAG_VALIDATION_THRESHOLD) {
          // Quick restore of previous calibration
          questInitialized = true;
          Logger.recordOutput(
              "PoseEstimator/Event",
              "Quest reconnected with valid calibration - resuming tracking");
          state.transitionTo(LocalizationState.State.QUEST_PRIMARY);
          return;
        } else {
          Logger.recordOutput(
              "PoseEstimator/Event", "Quest calibration invalid after reconnect - reinitializing");
          hadPreviousCalibration = false;
        }
      }
    }

    // Otherwise proceed with full initialization
    Pose2d questPose = oculusSource.getCurrentPose();
    if (questPose == null) {
      return;
    }

    // Start initialization process
    if (questInitStartTime == 0.0) {
      questInitStartTime = Timer.getTimestamp();
      lastQuestInitPose = questPose;

      // If we have a valid reference pose, use it
      Pose2d referencePose = getValidReferencePose();
      if (referencePose != null) {
        if (!resetToPose(referencePose)) {
          Logger.recordOutput(
              "PoseEstimator/Event", "Quest initialization failed - could not set initial pose");
          resetQuestInitialization();
          return;
        }
      }

      state.transitionTo(LocalizationState.State.RESETTING);
      return;
    }

    // Verify pose stability and consistency
    if (isNewPoseValid(questPose, lastQuestInitPose, INIT_VALIDATION_THRESHOLD)) {
      consecutiveValidQuestUpdates++;
      lastQuestInitPose = questPose;
    } else {
      consecutiveValidQuestUpdates = 0;
    }

    // Check initialization criteria
    double initDuration = Timer.getTimestamp() - questInitStartTime;
    if (consecutiveValidQuestUpdates >= MIN_QUEST_VALID_UPDATES
        && initDuration >= QUEST_INIT_TIMEOUT) {

      questInitialized = true;
      Logger.recordOutput("PoseEstimator/Event", "Quest initialization complete - system ready");

      // Only switch to primary if AprilTags aren't providing better data
      if (!tagInitialized || shouldPreferQuest()) {
        state.transitionTo(LocalizationState.State.QUEST_PRIMARY);
      }
    } else if (initDuration > QUEST_INIT_TIMEOUT * 2) {
      Logger.recordOutput("PoseEstimator/Event", "Quest initialization failed - timeout exceeded");
      resetQuestInitialization();
    }
  }

  /** Handles initialization of the AprilTag vision system. */
  private void handleTagInitialization() {
    if (!tagSource.isConnected()) {
      resetTagInitialization();
      return;
    }

    Pose2d tagPose = tagSource.getCurrentPose();
    if (tagPose == null) {
      return;
    }

    // Start initialization process
    if (tagInitStartTime == 0.0) {
      tagInitStartTime = Timer.getTimestamp();
      lastTagInitPose = tagPose;
      return;
    }

    // Verify pose stability and consistency
    if (isNewPoseValid(tagPose, lastTagInitPose, INIT_VALIDATION_THRESHOLD)) {
      consecutiveValidTagUpdates++;
      lastTagInitPose = tagPose;
    } else {
      consecutiveValidTagUpdates = 0;
    }

    // Check initialization criteria
    double initDuration = Timer.getTimestamp() - tagInitStartTime;
    if (consecutiveValidTagUpdates >= MIN_TAG_VALID_UPDATES && initDuration >= TAG_INIT_TIMEOUT) {

      tagInitialized = true;
      Logger.recordOutput("PoseEstimator/Event", "AprilTag initialization complete - system ready");

      // Switch to TAG_BACKUP if Quest isn't ready or is less reliable
      if (!questInitialized || !shouldPreferQuest()) {
        state.transitionTo(LocalizationState.State.TAG_BACKUP);
      }
    } else if (initDuration > TAG_INIT_TIMEOUT * 2) {
      Logger.recordOutput(
          "PoseEstimator/Event", "AprilTag initialization failed - timeout exceeded");
      resetTagInitialization();
    }
  }

  /** Updates pose estimates based on current state and available sources. */
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
            "PoseEstimator/Event", "No valid pose sources available - emergency mode active");
        break;
    }
  }

  private boolean shouldPreferQuest() {
    if (!questInitialized) return false;
    if (!tagInitialized) return true;

    // If we're in a mid-match initialization (DriverStation is enabled and enough time has passed)
    boolean isMidMatch = DriverStation.isEnabled() && Timer.getMatchTime() > 5.0;

    // During mid-match initialization, prefer AprilTags until Quest fully validates
    if (isMidMatch && Timer.getTimestamp() - questInitStartTime < QUEST_INIT_TIMEOUT * 3) {
      return false;
    }

    // Otherwise strongly prefer Quest as primary source
    return validatePose(oculusSource.getCurrentPose());
  }

  private boolean validatePose(Pose2d pose) {
    if (pose == null) return false;

    Pose2d tagPose = tagSource.getCurrentPose();
    if (tagPose == null) {
      return true; // No validation possible, trust Quest
    }

    double poseError = tagPose.getTranslation().getDistance(pose.getTranslation());

    // During initialization, use stricter threshold
    if (!questInitialized) {
      return poseError <= INIT_VALIDATION_THRESHOLD;
    }

    // Once initialized, be more lenient with Quest
    return poseError <= APRILTAG_VALIDATION_THRESHOLD;
  }

  private boolean isNewPoseValid(Pose2d newPose, Pose2d lastPose, double maxChange) {
    if (lastPose == null) return true;

    double poseChange = newPose.getTranslation().getDistance(lastPose.getTranslation());
    double rotationChange =
        Math.abs(newPose.getRotation().minus(lastPose.getRotation()).getDegrees());

    return poseChange <= maxChange && rotationChange <= 45.0; // 45 degrees max rotation
  }

  private Pose2d getValidReferencePose() {
    if (tagInitialized) {
      return tagSource.getCurrentPose();
    }
    return null;
  }

  private void resetQuestInitialization() {
    questInitStartTime = 0.0;
    consecutiveValidQuestUpdates = 0;
    lastQuestInitPose = null;
    questInitialized = false;
  }

  private void resetTagInitialization() {
    tagInitStartTime = 0.0;
    consecutiveValidTagUpdates = 0;
    lastTagInitPose = null;
    tagInitialized = false;
  }

  private void logDetailedStatus() {
    Logger.recordOutput("PoseEstimator/QuestInitialized", questInitialized);
    Logger.recordOutput("PoseEstimator/TagInitialized", tagInitialized);
    Logger.recordOutput("PoseEstimator/QuestHadCalibration", hadPreviousCalibration);

    if (questInitialized) {
      Logger.recordOutput("PoseEstimator/QuestValidUpdates", consecutiveValidQuestUpdates);
      Logger.recordOutput("PoseEstimator/LastQuestUpdate", lastQuestUpdate);
    }
    if (tagInitialized) {
      Logger.recordOutput("PoseEstimator/TagValidUpdates", consecutiveValidTagUpdates);
    }
  }

  /* Original state handling methods remain unchanged */
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

  private void handleQuestPrimaryState(double currentTime) {
    if (!oculusSource.isConnected()) {
      if (questInitialized) {
        // Mark for potential quick reconnect
        lastQuestDisconnectTime = Timer.getTimestamp();
        hadPreviousCalibration = true;
        lastQuestInitPose = oculusSource.getCurrentPose();
      }
      state.transitionTo(LocalizationState.State.TAG_BACKUP);
      Logger.recordOutput("PoseEstimator/Event", "Quest connection lost - switching to AprilTags");
      return;
    }

    Pose2d questPose = oculusSource.getCurrentPose();
    if (questPose != null) {
      if (validatePose(questPose)) {
        poseConsumer.accept(questPose, currentTime, oculusSource.getStdDevs());
        lastQuestUpdate = currentTime;
        lastValidatedPose = questPose;
      } else {
        Logger.recordOutput("PoseEstimator/Event", "Quest pose validation failed");
        // Don't immediately switch to backup if we have recent valid data
        if (currentTime - lastQuestUpdate > QUEST_INIT_TIMEOUT) {
          state.transitionTo(LocalizationState.State.TAG_BACKUP);
        }
      }
    }
  }

  private void handleTagBackupState(double currentTime) {
    // Check for Quest quick reconnect opportunity
    if (oculusSource.isConnected()
        && hadPreviousCalibration
        && (Timer.getTimestamp() - lastQuestDisconnectTime) < QUICK_RECONNECT_WINDOW) {

      Pose2d questPose = oculusSource.getCurrentPose();
      if (questPose != null && lastQuestInitPose != null) {
        double poseJump =
            questPose.getTranslation().getDistance(lastQuestInitPose.getTranslation());

        if (poseJump < APRILTAG_VALIDATION_THRESHOLD) {
          state.transitionTo(LocalizationState.State.QUEST_PRIMARY);
          Logger.recordOutput(
              "PoseEstimator/Event",
              "Quest connection restored with valid calibration - resuming primary");
          return;
        }
      }
    }

    // If Quest is available but needs full reinit
    if (oculusSource.isConnected() && !hadPreviousCalibration) {
      handleQuestInitialization();
      return;
    }

    // Otherwise use AprilTag data
    Pose2d tagPose = tagSource.getCurrentPose();
    if (tagPose != null) {
      poseConsumer.accept(tagPose, currentTime, tagSource.getStdDevs());
      lastValidatedPose = tagPose;
    }
  }

  private void logSystemStatus() {
    Logger.recordOutput("PoseEstimator/State", state.getCurrentState().getDescription());
    Logger.recordOutput("PoseEstimator/OculusConnected", oculusSource.isConnected());
    Logger.recordOutput("PoseEstimator/AprilTagConnected", tagSource.isConnected());
  }

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

  private boolean resetToPose(Pose2d pose) {
    return oculusSource.subsystem.resetToPose(pose);
  }

  private boolean isResetInProgress() {
    return oculusSource.subsystem.isPoseResetInProgress();
  }

  /**
   * Manually triggers a pose reset using the most recent validated pose.
   *
   * @return true if reset was initiated successfully
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
   * Manually triggers a pose reset to a specific pose.
   *
   * @param targetPose The pose to reset to
   * @return true if reset was initiated successfully
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

  /** Updates system state based on source availability and initialization status */
  private void updateState() {
    // Skip state updates during initialization phases
    if (state.isInState(LocalizationState.State.UNINITIALIZED, LocalizationState.State.RESETTING)) {
      return;
    }

    // Handle source availability
    if (oculusSource.isConnected() && questInitialized) {
      // If we have a good Quest connection and it's initialized, prefer it
      state.transitionTo(LocalizationState.State.QUEST_PRIMARY);
    } else if (tagSource.isConnected() && tagInitialized) {
      // Fall back to AprilTags if available and initialized
      state.transitionTo(LocalizationState.State.TAG_BACKUP);
    } else {
      // No valid sources available
      state.transitionTo(LocalizationState.State.EMERGENCY);
      Logger.recordOutput(
          "PoseEstimator/Event", "No valid pose sources available - entering emergency mode");
    }
  }

  @Override
  public void logTransition(LocalizationState.State from, LocalizationState.State to) {
    Logger.recordOutput(
        "PoseEstimator/StateTransition",
        String.format("State transition: %s -> %s", from.name(), to.name()));
  }
}
