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

import static frc.alotobots.library.subsystems.vision.localizationfusion.LocalizationFusionConstants.InitializationRequirements.*;
import static frc.alotobots.library.subsystems.vision.localizationfusion.LocalizationFusionConstants.Timing.*;
import static frc.alotobots.library.subsystems.vision.localizationfusion.LocalizationFusionConstants.ValidationThresholds.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.vision.oculus.util.OculusPoseSource;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.util.AprilTagPoseSource;
import org.littletonrobotics.junction.Logger;

/**
 * The LocalizationFusion subsystem manages robot pose estimation by fusing data from multiple
 * sources. It primarily uses Oculus Quest SLAM for continuous tracking, with AprilTag detection as
 * a backup and validation source. The system handles initialization, validation, and switching
 * between sources based on their availability and reliability.
 */
public class LocalizationFusion extends SubsystemBase implements StateTransitionLogger {

  // -------------------- Dependencies --------------------
  /** Interface for consuming pose updates from the localization system. */
  private final PoseVisionConsumer poseConsumer;

  /** Primary pose source using Oculus Quest SLAM tracking. */
  private final OculusPoseSource oculusSource;

  /** Secondary pose source using AprilTag vision tracking. */
  private final AprilTagPoseSource tagSource;

  /** State machine managing transitions between different localization states. */
  private final LocalizationState state;

  /** Tab for drivers to ensure readiness */
  private final ShuffleboardTab prematchTab = Shuffleboard.getTab("Prematch");

  // -------------------- State Variables --------------------
  /** Timestamp of the last pose update processed. */
  private double lastUpdateTime = 0.0;

  /** Timestamp of the last valid Quest pose update. */
  private double lastQuestUpdate = 0.0;

  /** Timestamp when the current reset sequence started. */
  private double resetStartTime = 0.0;

  // Connection State
  /** Previous DriverStation connection state. */
  private boolean wasConnected = false;

  /** Tracks whether we've had our initial DriverStation connection. */
  private boolean hadInitialConnection = false;

  /** Indicates if an initial reset is needed due to pre-connected DriverStation. */
  private boolean needsInitialReset = false;

  // Quest State
  /** Timestamp when Quest initialization started. */
  private double questInitStartTime = 0.0;

  /** Counter for consecutive valid Quest pose updates during initialization. */
  private int consecutiveValidQuestUpdates = 0;

  /** Last valid Quest pose during initialization. */
  private Pose2d lastQuestInitPose = null;

  /** Indicates if Quest initialization is complete. */
  private boolean questInitialized = false;

  /** Timestamp of the last Quest disconnection. */
  private double lastQuestDisconnectTime = 0.0;

  /** Indicates if Quest had a previous valid calibration before disconnection. */
  private boolean hadPreviousCalibration = false;

  // AprilTag State
  /** Timestamp when AprilTag initialization started. */
  private double tagInitStartTime = 0.0;

  /** Counter for consecutive valid AprilTag pose updates during initialization. */
  private int consecutiveValidTagUpdates = 0;

  /** Last valid AprilTag pose during initialization. */
  private Pose2d lastTagInitPose = null;

  /** Indicates if AprilTag initialization is complete. */
  private boolean tagInitialized = false;

  // Pose Validation
  /** Timestamp when initial pose validation started. */
  private double initialPoseValidationStartTime = 0.0;

  /** Last validated pose during initialization or disabled state. */
  private Pose2d lastValidatedPose = null;

  /** Indicates if initial pose validation is complete. */
  private boolean initialPoseValidated = false;

  

  // -------------------- Constructor --------------------
  /**
   * Creates a new LocalizationFusion subsystem.
   *
   * @param poseConsumer The consumer interface that will receive pose updates
   * @param oculusSource The primary pose source using Quest SLAM
   * @param aprilTagSource The secondary pose source using AprilTags
   */
  public LocalizationFusion(
      PoseVisionConsumer poseConsumer,
      OculusPoseSource oculusSource,
      AprilTagPoseSource aprilTagSource) {
    this.poseConsumer = poseConsumer;
    this.oculusSource = oculusSource;
    this.tagSource = aprilTagSource;
    this.state = new LocalizationState(this);
    initializeConnectionState();
    setupShuffleboardLogging();
  }

  // -------------------- Core System Methods --------------------
  /**
   * Sets up Shuffleboard logging for critical system status indicators in the /Prematch tab.
   * Configures three status indicators arranged vertically:
   *
   * <p>1. "Quest Ready" - Indicates if the Oculus Quest SLAM system is connected and initialized
   *
   * <p>2. "Tags Ready" - Indicates if the AprilTag vision system is connected and initialized
   *
   * <p>3. "System Ready" - Indicates if both localization systems are ready and not in emergency
   * state
   *
   * <p>Each indicator is configured with a size of 2x1 grid units and positioned vertically in the
   * Prematch tab. The indicators automatically update based on system state changes and will show
   * as green when true and red when false.
   *
   * <p>Layout:
   *
   * <pre>
   * [Quest Ready]
   * [Tags Ready]
   * [System Ready]
   * </pre>
   */
  private void setupShuffleboardLogging() {
    prematchTab
        .addBoolean("Quest Ready", () -> oculusSource.isConnected() && questInitialized)
        .withSize(2, 1)
        .withPosition(0, 0);

    prematchTab
        .addBoolean("Tags Ready", () -> tagSource.isConnected() && tagInitialized)
        .withSize(2, 1)
        .withPosition(0, 1);

    prematchTab
        .addBoolean(
            "System Ready",
            () ->
                (oculusSource.isConnected() && questInitialized)
                    && (tagSource.isConnected() && tagInitialized)
                    && state.getCurrentState() != LocalizationState.State.EMERGENCY)
        .withSize(2, 1)
        .withPosition(0, 2);
  }

  /**
   * Periodic update method called by the command scheduler. Handles system state updates, source
   * initialization, and pose processing.
   */
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

    // State-specific handling
    double currentTime = Timer.getTimestamp();
    switch (state.getCurrentState()) {
      case RESETTING:
        handleResettingState();
        break;
      case EMERGENCY:
        handleEmergencyState();
        break;
      case QUEST_PRIMARY:
        handleQuestPrimaryState(currentTime);
        break;
      case TAG_BACKUP:
        handleTagBackupState(currentTime);
        break;
      case UNINITIALIZED:
        handleUninitializedState();
        break;
    }

    logDetailedStatus();
  }

  // -------------------- State Handlers --------------------
  /**
   * Handles the RESETTING state where the system is attempting to align Quest and AprilTag poses.
   * Monitors timeout conditions and validates pose alignment before transitioning to primary state.
   */
  private void handleResettingState() {
    if (resetStartTime > 0 && Timer.getTimestamp() - resetStartTime > RESET_TIMEOUT) {
      Logger.recordOutput(
          "LocalizationFusion/Event", "Reset sequence timed out - switching to emergency mode");
      resetStartTime = 0.0;
      resetQuestInitialization();
      state.transitionTo(LocalizationState.State.EMERGENCY);
      return;
    }

    Pose2d tagPose = tagSource.getCurrentPose();
    if (tagPose == null) {
      if (resetStartTime == 0.0) {
        resetStartTime = Timer.getTimestamp();
      }
      return;
    }

    if (!oculusSource.isConnected() || !tagSource.isConnected()) {
      return;
    }

    if (isResetInProgress()) {
      return;
    }

    Pose2d questPose = oculusSource.getCurrentPose();
    if (questPose == null) {
      return;
    }

    double poseError = questPose.getTranslation().getDistance(tagPose.getTranslation());
    if (poseError <= INIT_VALIDATION_THRESHOLD) {
      resetStartTime = 0.0;
      lastQuestUpdate = Timer.getTimestamp();
      state.transitionTo(LocalizationState.State.QUEST_PRIMARY);
    }
  }

  /**
   * Handles the EMERGENCY state where normal operation has failed. Attempts to recover using
   * AprilTag detection and reinitialize the system.
   */
  private void handleEmergencyState() {
    if (tagSource.isConnected()) {
      Pose2d tagPose = tagSource.getCurrentPose();
      if (tagPose != null) {
        if (oculusSource.isConnected() && resetToPose(tagPose)) {
          resetStartTime = Timer.getTimestamp();
          state.transitionTo(LocalizationState.State.RESETTING);
        } else {
          state.transitionTo(LocalizationState.State.TAG_BACKUP);
        }
      }
    }
  }

  /**
   * Handles the QUEST_PRIMARY state where Quest SLAM is the primary pose source. Validates poses
   * and handles Quest disconnection scenarios.
   *
   * @param currentTime The current system timestamp
   */
  private void handleQuestPrimaryState(double currentTime) {
    if (currentTime - lastUpdateTime < POSE_UPDATE_INTERVAL) {
      return;
    }
    lastUpdateTime = currentTime;

    if (!oculusSource.isConnected()) {
      if (questInitialized) {
        lastQuestDisconnectTime = Timer.getTimestamp();
        hadPreviousCalibration = true;
        lastQuestInitPose = oculusSource.getCurrentPose();
      }
      state.transitionTo(LocalizationState.State.TAG_BACKUP);
      return;
    }

    Pose2d questPose = oculusSource.getCurrentPose();
    if (questPose != null) {
      if (validatePose(questPose, false)) {
        poseConsumer.accept(questPose, currentTime, oculusSource.getStdDevs());
        lastQuestUpdate = currentTime;
      } else {
        if (currentTime - lastQuestUpdate > QUEST_INIT_TIMEOUT) {
          state.transitionTo(LocalizationState.State.TAG_BACKUP);
        }
      }
    }
  }

  /**
   * Handles the TAG_BACKUP state where AprilTag detection is the primary pose source. Attempts
   * Quest reconnection when possible and processes AprilTag poses.
   *
   * @param currentTime The current system timestamp
   */
  private void handleTagBackupState(double currentTime) {
    if (currentTime - lastUpdateTime < POSE_UPDATE_INTERVAL) {
      return;
    }
    lastUpdateTime = currentTime;

    if (oculusSource.isConnected() && hadPreviousCalibration) {

      Pose2d questPose = oculusSource.getCurrentPose();
      if (validatePose(questPose, true)) {
        state.transitionTo(LocalizationState.State.QUEST_PRIMARY);
        Logger.recordOutput(
            "LocalizationFusion/Event",
            "Quest Reconnected; Pose Validated - maintaining Quest tracking");
        return;
      } else {
        // Re-init as we couldn't validate pose
        Logger.recordOutput(
            "LocalizationFusion/Event",
            "Quest Reconnected; Pose Failed to Validate - reinitializing Quest tracking");
        handleQuestInitialization();
      }
    }

    if (oculusSource.isConnected() && !hadPreviousCalibration) {
      handleQuestInitialization();
      return;
    }

    Pose2d tagPose = tagSource.getCurrentPose();
    if (tagPose != null) {
      poseConsumer.accept(tagPose, currentTime, tagSource.getStdDevs());
    }
  }

  /**
   * Handles the UNINITIALIZED state where the system is starting up. Attempts to initialize using
   * AprilTag detection.
   */
  private void handleUninitializedState() {
    Pose2d tagPose = tagSource.getCurrentPose();
    if (tagPose != null) {
      if (DriverStation.isDSAttached() && resetToPose(tagPose)) {
        state.transitionTo(LocalizationState.State.RESETTING);
      }
    }
  }

  // -------------------- Initialization Methods --------------------
  /**
   * Handles the initialization of the Quest SLAM system. Manages calibration, validation, and state
   * transitions during Quest startup.
   */
  private void handleQuestInitialization() {
    if (!oculusSource.isConnected()) {
      if (questInitialized) {
        lastQuestDisconnectTime = Timer.getTimestamp();
        hadPreviousCalibration = true;
        lastQuestInitPose = oculusSource.getCurrentPose();
      }
      resetQuestInitialization();
      return;
    }

    // Skip validation during mid-match reboot
    if (isMidMatchReboot()) {
      questInitialized = true;
      lastQuestUpdate = Timer.getTimestamp();
      state.transitionTo(LocalizationState.State.QUEST_PRIMARY);
      return;
    }

    Pose2d questPose = oculusSource.getCurrentPose();
    if (questPose == null) {
      return;
    }

    if (questInitStartTime == 0.0) {
      questInitStartTime = Timer.getTimestamp();
      lastQuestInitPose = questPose;
      return;
    }

    if (isNewPoseValid(questPose, lastQuestInitPose, INIT_VALIDATION_THRESHOLD)) {
      consecutiveValidQuestUpdates++;
      lastQuestInitPose = questPose;
    } else {
      consecutiveValidQuestUpdates = 0;
    }

    double initDuration = Timer.getTimestamp() - questInitStartTime;
    if (consecutiveValidQuestUpdates >= MIN_QUEST_VALID_UPDATES
        && initDuration >= QUEST_INIT_TIMEOUT) {
      questInitialized = true;
      Pose2d referencePose = getValidReferencePose();
      if (referencePose != null && resetToPose(referencePose)) {
        state.transitionTo(LocalizationState.State.RESETTING);
        resetStartTime = Timer.getTimestamp();
      }
    } else if (initDuration > QUEST_INIT_TIMEOUT * QUEST_INIT_GRACE_MULTIPLIER) {
      resetQuestInitialization();
    }
  }

  /**
   * Handles the initialization of the AprilTag vision system. Manages detection validation and
   * state transitions during AprilTag startup.
   */
  private void handleTagInitialization() {
    if (!tagSource.isConnected()) {
      resetTagInitialization();
      return;
    }

    Pose2d tagPose = tagSource.getCurrentPose();
    if (tagPose == null) {
      return;
    }

    if (tagInitStartTime == 0.0) {
      tagInitStartTime = Timer.getTimestamp();
      lastTagInitPose = tagPose;
      return;
    }

    if (isNewPoseValid(tagPose, lastTagInitPose, INIT_VALIDATION_THRESHOLD)) {
      consecutiveValidTagUpdates++;
      lastTagInitPose = tagPose;
    } else {
      consecutiveValidTagUpdates = 0;
    }

    double initDuration = Timer.getTimestamp() - tagInitStartTime;
    if (consecutiveValidTagUpdates >= MIN_TAG_VALID_UPDATES && initDuration >= TAG_INIT_TIMEOUT) {
      tagInitialized = true;
      if (!questInitialized || !shouldPreferQuest()) {
        state.transitionTo(LocalizationState.State.TAG_BACKUP);
      }
    } else if (initDuration > TAG_INIT_TIMEOUT * QUEST_INIT_GRACE_MULTIPLIER) {
      resetTagInitialization();
    }
  }

  // -------------------- Connection Management --------------------
  /**
   * Checks if we're rebooting during an active match by examining match time. A non-zero match time
   * indicates the FMS->DS->Robot connection is active and we're in a match that was already
   * running.
   *
   * @return true if this appears to be a mid-match reboot
   */
  private boolean isMidMatchReboot() {
    return DriverStation.isEnabled() && Timer.getMatchTime() > 0;
  }

  /**
   * Initializes the DriverStation connection state tracking. Called during subsystem construction
   * to handle pre-connected scenarios and mid-match reboots.
   */
  private void initializeConnectionState() {
    wasConnected = DriverStation.isDSAttached();
    hadInitialConnection = wasConnected;

    if (isMidMatchReboot()) {
      Logger.recordOutput(
          "LocalizationFusion/Event", "Mid-match reboot detected - maintaining Quest tracking");
      // Skip initialization and trust Quest's maintained tracking
      questInitialized = true;
      hadPreviousCalibration = true;

      if (oculusSource.isConnected()) {
        // Go straight to Quest primary if available
        state.transitionTo(LocalizationState.State.QUEST_PRIMARY);
      } else {
        // Fallback to tags if Quest isn't ready yet
        state.transitionTo(LocalizationState.State.TAG_BACKUP);
      }
    } else if (hadInitialConnection) {
      state.transitionTo(LocalizationState.State.UNINITIALIZED);
      needsInitialReset = true;
    }
  }

  /**
   * Handles DriverStation connection state changes and triggers appropriate system responses.
   * Manages initial connection validation and pose recalibration during disabled state.
   */
  private void handleDriverStationConnection() {
    boolean isConnected = DriverStation.isDSAttached();

    if (!wasConnected && isConnected) {
      if (!hadInitialConnection) {
        Pose2d tagPose = tagSource.getCurrentPose();
        if (tagPose != null) {
          lastValidatedPose = tagPose;
          initialPoseValidationStartTime = Timer.getTimestamp();
          initialPoseValidated = false;
        }
        hadInitialConnection = true;
      } else {
        Logger.recordOutput(
            "LocalizationFusion/Event", "DriverStation reconnected - maintaining current pose");
      }
    }

    if (isConnected && DriverStation.isDisabled()) {
      handleDisabledPoseValidation();
    }

    wasConnected = isConnected;
  }

  // -------------------- Validation Methods --------------------
  /**
   * Handles pose validation during the disabled state. Monitors pose stability and triggers
   * recalibration if significant changes are detected.
   */
  private void handleDisabledPoseValidation() {
    Pose2d currentTagPose = tagSource.getCurrentPose();
    if (currentTagPose == null) return;

    if (!initialPoseValidated) {
      if (Timer.getTimestamp() - initialPoseValidationStartTime >= INITIAL_POSE_STABILITY_TIME) {
        if (lastValidatedPose != null) {
          double poseChange =
              currentTagPose.getTranslation().getDistance(lastValidatedPose.getTranslation());

          if (poseChange <= INIT_VALIDATION_THRESHOLD) {
            Logger.recordOutput(
                "LocalizationFusion/Event",
                "Initial pose validated after stability period - initiating reset");
            if (resetToPose(currentTagPose)) {
              state.transitionTo(LocalizationState.State.RESETTING);
              initialPoseValidated = true;
            }
          } else {
            Logger.recordOutput(
                "LocalizationFusion/Event",
                "Pose changed during validation - restarting stability timer");
            initialPoseValidationStartTime = Timer.getTimestamp();
            lastValidatedPose = currentTagPose;
          }
        }
      }
      lastValidatedPose = currentTagPose;
      return;
    }

    if (lastValidatedPose != null) {
      double poseChange =
          currentTagPose.getTranslation().getDistance(lastValidatedPose.getTranslation());

      if (poseChange >= DISABLED_RECALIBRATION_THRESHOLD) {
        Logger.recordOutput(
            "LocalizationFusion/Event",
            String.format(
                "Significant pose change detected while disabled (%.2fm) - recalibrating",
                poseChange));

        initialPoseValidationStartTime = Timer.getTimestamp();
        lastValidatedPose = currentTagPose;
        initialPoseValidated = false;
      }
    }

    lastValidatedPose = currentTagPose;
  }

  /**
   * Validates a pose against the current AprilTag reference. Uses different thresholds for
   * initialization and normal operation.
   *
   * @param pose The pose to validate
   * @param strict True if we require tags to be present for validation, false to assume valid with
   *     no tags
   * @return true if the pose is valid, false otherwise
   */
  private boolean validatePose(Pose2d pose, boolean strict) {
    if (pose == null) return false;

    Pose2d tagPose = tagSource.getCurrentPose();
    if (tagPose == null) {
      return !strict;
    }

    double poseError = tagPose.getTranslation().getDistance(pose.getTranslation());

    if (!questInitialized) {
      return poseError <= INIT_VALIDATION_THRESHOLD;
    }

    return poseError <= APRILTAG_VALIDATION_THRESHOLD;
  }

  /**
   * Validates a new pose against a previous pose using maximum change thresholds.
   *
   * @param newPose The new pose to validate
   * @param lastPose The previous pose to compare against
   * @param maxChange The maximum allowed position change in meters
   * @return true if the pose change is within acceptable limits
   */
  private boolean isNewPoseValid(Pose2d newPose, Pose2d lastPose, double maxChange) {
    if (lastPose == null) return true;

    double poseChange = newPose.getTranslation().getDistance(lastPose.getTranslation());
    double rotationChange =
        Math.abs(newPose.getRotation().minus(lastPose.getRotation()).getDegrees());

    return poseChange <= maxChange && rotationChange <= MAX_ROTATION_CHANGE_DEGREES;
  }

  /**
   * Gets a valid reference pose from the AprilTag system if available.
   *
   * @return A valid reference Pose2d or null if unavailable
   */
  private Pose2d getValidReferencePose() {
    if (tagInitialized) {
      return tagSource.getCurrentPose();
    }
    return null;
  }

  // -------------------- Reset Methods --------------------
  /**
   * Attempts to reset the Quest pose to match a target pose.
   *
   * @param pose The target pose to reset to
   * @return true if reset was initiated successfully
   */
  private boolean resetToPose(Pose2d pose) {
    return oculusSource.subsystem.resetToPose(pose);
  }

  /**
   * Checks if a pose reset operation is currently in progress.
   *
   * @return true if a reset is in progress
   */
  private boolean isResetInProgress() {
    return oculusSource.subsystem.isPoseResetInProgress();
  }

  /**
   * Manually triggers a pose reset using the current AprilTag pose.
   *
   * @return true if reset was initiated successfully
   */
  public boolean requestResetOculusPoseViaAprilTags() {
    Pose2d currentTagPose = tagSource.getCurrentPose();
    if (currentTagPose == null) {
      Logger.recordOutput(
          "LocalizationFusion/Event", "Manual reset failed - no AprilTag pose available");
      return false;
    }

    Logger.recordOutput(
        "LocalizationFusion/Event",
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
      Logger.recordOutput("LocalizationFusion/Event", "Manual reset failed - null pose provided");
      return false;
    }

    Logger.recordOutput(
        "LocalizationFusion/Event",
        "Manual reset requested with custom pose - initiating pose reset");
    if (resetToPose(targetPose)) {
      state.transitionTo(LocalizationState.State.RESETTING);
      return true;
    }
    return false;
  }

  // -------------------- State Management Helpers --------------------
  /** Resets all Quest initialization state variables. */
  private void resetQuestInitialization() {
    questInitStartTime = 0.0;
    consecutiveValidQuestUpdates = 0;
    lastQuestInitPose = null;
    questInitialized = false;
  }

  /** Resets all AprilTag initialization state variables. */
  private void resetTagInitialization() {
    tagInitStartTime = 0.0;
    consecutiveValidTagUpdates = 0;
    lastTagInitPose = null;
    tagInitialized = false;
  }

  /**
   * Determines whether Quest should be preferred as the primary pose source. Considers
   * initialization status, match state, and pose validity.
   *
   * @return true if Quest should be the primary source
   */
  private boolean shouldPreferQuest() {
    if (!questInitialized) return false;
    if (!tagInitialized) return true;

    boolean isMidMatch =
        DriverStation.isEnabled() && Timer.getMatchTime() > MATCH_STARTUP_PERIOD_SECONDS;

    if (isMidMatch
        && Timer.getTimestamp() - questInitStartTime
            < QUEST_INIT_TIMEOUT * QUEST_INIT_GRACE_MULTIPLIER) {
      return false;
    }

    return validatePose(oculusSource.getCurrentPose(), false);
  }

  // -------------------- Logging Methods --------------------
  /** Logs basic system status information including state and connection status. */
  private void logSystemStatus() {
    Logger.recordOutput("LocalizationFusion/State", state.getCurrentState().getDescription());
    Logger.recordOutput("LocalizationFusion/OculusConnected", oculusSource.isConnected());
    Logger.recordOutput("LocalizationFusion/AprilTagConnected", tagSource.isConnected());
  }

  /** Logs detailed system status including initialization states and update counts. */
  private void logDetailedStatus() {
    Logger.recordOutput("LocalizationFusion/QuestInitialized", questInitialized);
    Logger.recordOutput("LocalizationFusion/TagInitialized", tagInitialized);
    Logger.recordOutput("LocalizationFusion/QuestHadCalibration", hadPreviousCalibration);

    if (questInitialized) {
      Logger.recordOutput("LocalizationFusion/QuestValidUpdates", consecutiveValidQuestUpdates);
      Logger.recordOutput("LocalizationFusion/LastQuestUpdate", lastQuestUpdate);
    }
    if (tagInitialized) {
      Logger.recordOutput("LocalizationFusion/TagValidUpdates", consecutiveValidTagUpdates);
    }
  }

  /**
   * Logs state transitions with descriptive messages.
   *
   * @param from The previous state
   * @param to The new state
   */
  @Override
  public void logTransition(LocalizationState.State from, LocalizationState.State to) {
    Logger.recordOutput(
        "LocalizationFusion/StateTransition",
        String.format("State transition: %s -> %s", from.name(), to.name()));
  }
}
