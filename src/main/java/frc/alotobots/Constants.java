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
package frc.alotobots;

import edu.wpi.first.wpilibj.RobotBase;
import frc.alotobots.library.subsystems.swervedrive.constants.TunerConstants;
import frc.alotobots.library.subsystems.swervedrive.constants.mk4i2023.TunerConstants2023;
import lombok.experimental.UtilityClass;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.REPLAY;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final TunerConstants tunerConstants = new TunerConstants2023();

  @UtilityClass
  /**
   * CAN bus device IDs CAN bus device ID assignments. Maps CAN IDs for motors, sensors and other
   * CAN-connected devices.
   */
  public static final class CanId {
    /** Power Distribution Panel CAN ID */
    public static final int PDP_CAN_ID = 1;

    /** Pneumatic Control Module CAN ID */
    public static final int PCM_CAN_ID = 2;

    /** Pigeon 2 IMU CAN ID */
    public static final int PIGEON_2_ID = 3;

    /** Front left drive motor CAN ID */
    public static final int FRONT_LEFT_DRIVE_MTR_CAN_ID = 10;

    /** Front left steering motor CAN ID */
    public static final int FRONT_LEFT_STEER_MTR_CAN_ID = 11;

    /** Front left steering encoder CAN ID */
    public static final int FRONT_LEFT_STEER_CAN_CODER_CAN_ID = 12;

    /** Front right drive motor CAN ID */
    public static final int FRONT_RIGHT_DRIVE_MTR_CAN_ID = 13;

    /** Front right steering motor CAN ID */
    public static final int FRONT_RIGHT_STEER_MTR_CAN_ID = 14;

    /** Front right steering encoder CAN ID */
    public static final int FRONT_RIGHT_STEER_CAN_CODER_CAN_ID = 15;

    /** Back left drive motor CAN ID */
    public static final int BACK_LEFT_DRIVE_MTR_CAN_ID = 16;

    /** Back left steering motor CAN ID */
    public static final int BACK_LEFT_STEER_MTR_CAN_ID = 17;

    /** Back left steering encoder CAN ID */
    public static final int BACK_LEFT_STEER_CAN_CODER_CAN_ID = 18;

    /** Back right drive motor CAN ID */
    public static final int BACK_RIGHT_DRIVE_MTR_CAN_ID = 19;

    /** Back right steering motor CAN ID */
    public static final int BACK_RIGHT_STEER_MTR_CAN_ID = 20;

    /** Back right steering encoder CAN ID */
    public static final int BACK_RIGHT_STEER_CAN_CODER_CAN_ID = 21;

    /** CANdle LED controller CAN ID */
    public static final int CANDLE_CAN_ID = 40;
  }
}
