package frc.alotobots;

import lombok.experimental.UtilityClass;

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

  @UtilityClass
  public static final class Robot {
    public static final boolean TUNE_MODE = false;

    @UtilityClass
    public static final class Calibrations {
      @UtilityClass
      public static final class DriveTrain {
        public static final double DRIVE_XY_SPD_PERF_MODE_SW_FILTER_RATE = 8.0; // m/s/s
        public static final double DRIVE_ROT_SPD_PERF_MODE_SW_FILTER_RATE = 4.0; // rad/s/s
        public static final double DRIVE_TRAIN_MAX_ACCEL = 3.0; // m/s^2
        public static final double DRIVE_TRAIN_MAX_DECCEL = -2.0; // m/s^2

        @UtilityClass
        public static final class PerformanceModeDefault {
          public static final double DRIVE_TRAIN_MAX_SPD = 2.0; // m/s
          public static final double DRIVE_TRAIN_MAX_ACCELERATION = 2.0; // m/s^2
          public static final double DRIVE_TRAIN_MAX_ROT_SPD = 0.75 * 2 * Math.PI; // rad/s
          public static final double DRIVE_TRAIN_MAX_ROT_ACCELERATION =
              0.75 * 2 * Math.PI; // rad/s^2
        }

        @UtilityClass
        public static final class PerformanceModeTurtle {
          public static final double DRIVE_TRAIN_MAX_SPD = 1.0; // m/s
          public static final double DRIVE_TRAIN_MAX_ACCELERATION = 0.35; // m/s^2
          public static final double DRIVE_TRAIN_MAX_ROT_SPD = 0.5 * 2 * Math.PI; // rad/s
          public static final double DRIVE_TRAIN_MAX_ROT_ACCELERATION =
              0.15 * 2 * Math.PI; // rad/s^2
        }

        @UtilityClass
        public static final class PerformanceModeTurbo {
          public static final double DRIVE_TRAIN_MAX_SPD = 10.0; // m/s
          public static final double DRIVE_TRAIN_MAX_ACCELERATION = 1.00; // m/s^2
          public static final double DRIVE_TRAIN_MAX_ROT_SPD = 1.0 * 2 * Math.PI; // rad/s
          public static final double DRIVE_TRAIN_MAX_ROT_ACCELERATION =
              0.75 * 2 * Math.PI; // rad/s^2
        }
      }
    }

    @UtilityClass
    public static final class Field {
      // Add field-specific constants here
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
