package frc.alotobots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.experimental.UtilityClass;

enum Mode {
  REAL, // Running on the real robot
  SIM, // Running in simulation
  REPLAY // Replaying from a log file
}

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
@UtilityClass
public final class Constants {
  public static final Mode currentMode = Mode.REAL; // Default to real robot

  @UtilityClass
  public static final class Robot {

    @UtilityClass
    public static final class Field {
      // Add field-specific constants here
      // Ex. for the 2024 Game Season
      public static final Pose2d RED_SPEAKER_FRONT =
          new Pose2d(15.275, 5.585, Rotation2d.fromDegrees(0));
    }

    @UtilityClass
    public static final class CanId {
      public static final int PDP_CAN_ID = 1; // Power Distribution Panel
      public static final int PCM_CAN_ID = 2; // Pneumatic Control Module
      public static final int PIGEON_2_ID = 3;

      public static final int FRONT_LEFT_DRIVE_MTR_CAN_ID = 10;
      public static final int FRONT_LEFT_STEER_MTR_CAN_ID = 11;
      public static final int FRONT_LEFT_STEER_CAN_CODER_CAN_ID = 12;
      public static final int FRONT_RIGHT_DRIVE_MTR_CAN_ID = 13;
      public static final int FRONT_RIGHT_STEER_MTR_CAN_ID = 14;
      public static final int FRONT_RIGHT_STEER_CAN_CODER_CAN_ID = 15;
      public static final int BACK_LEFT_DRIVE_MTR_CAN_ID = 16;
      public static final int BACK_LEFT_STEER_MTR_CAN_ID = 17;
      public static final int BACK_LEFT_STEER_CAN_CODER_CAN_ID = 18;
      public static final int BACK_RIGHT_DRIVE_MTR_CAN_ID = 19;
      public static final int BACK_RIGHT_STEER_MTR_CAN_ID = 20;
      public static final int BACK_RIGHT_STEER_CAN_CODER_CAN_ID = 21;

      public static final int CANDLE_CAN_ID = 40;
    }

    @UtilityClass
    public static final class AnalogInputIds {
      // Add analog input IDs here
    }

    @UtilityClass
    public static final class DigitalIoIds {
      // Add digital I/O IDs here
    }

    @UtilityClass
    public static final class PwmIds {
      // Add PWM IDs here
    }
  }
}
