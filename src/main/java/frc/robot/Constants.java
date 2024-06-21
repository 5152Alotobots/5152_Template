/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class Robot {

    public static final class Calibrations {

      public static final class DriveTrain {
        public static final double driveXYSpdPerfModeSwFilterRate = 8.0; // m/s/s
        public static final double driveRotSpdPerfModeSwFilterRate = 4.0; // rad/s/s
        public static final double DriveTrainMaxAccel = 3.0; // m/s^2
        public static final double DriveTrainMaxDeccel = -2.0; // m/s^2

        public static final class PerformanceMode_Default {
          // Default Performance Mode Speeds
          //public static double DriveTrainMaxPctOutput = 0.50; // 0-1
          public static final double DriveTrainMaxSpd = 2.0; // m/s
          public static double DriveTrainMaxAccel = 2.0; // m/s^2
          //public static double DriveTrainMaxRotPctOutput = 0.4; // 0-1
          public static final double DriveTrainMaxRotSpd = 0.75 * 2 * Math.PI; // rad/s
          public static double DriveTrainMaxRotAccel = 0.75 * 2 * Math.PI; // rad/s^2
        }

        public static final class PerformanceMode_Turtle {
          // Performance Mode A Speeds (Slow)
          //public static double DriveTrainMaxPctOutput = 0.25; // 0-1
          public static final double DriveTrainMaxSpd = 1.0; // m/s
          public static double DriveTrainMaxAccel = 0.35; // m/s^2
          //public static double DriveTrainMaxRotPctOutput = 0.6; // 0-1
          public static final double DriveTrainMaxRotSpd = 0.5 * 2 * Math.PI; // rad/s
          public static double DriveTrainMaxRotAccel = 0.15 * 2 * Math.PI; // rad/s^2
        }

        public static final class PerformanceMode_Turbo {
          // Performance Mode B Speeds (Fast)
          //public static double DriveTrainMaxPctOutput = 0.75; // 0-1
          public static final double DriveTrainMaxSpd = 10.0; // m/s
          public static double DriveTrainMaxAccel = 1.00; // m/s^2
          //public static double DriveTrainMaxRotPctOutput = 0.2; // 0-1
          public static final double DriveTrainMaxRotSpd = 1.0 * 2 * Math.PI; // rad/s
          public static double DriveTrainMaxRotAccel = 0.75 * 2 * Math.PI; // rad/s^2
        }
      }
    }

    public static final class Field {}

    public static final class CanId {

      /**
       * *** Start Library Components CAN ID's ****
       */
      public static final int PDP_CAN_ID = 1;   // Power Distribution Panel
      public static final int PCM_CAN_ID = 2;   // Pneumatic Control Module
      public static final int Pigeon2_ID = 3;

      public static final int FrontLeftDriveMtr_CAN_ID = 10;
      public static final int FrontLeftSteerMtr_CAN_ID = 11;
      public static final int FrontLeftSteerCANCoder_CAN_ID = 12;
      public static final int FrontRightDriveMtr_CAN_ID = 13;
      public static final int FrontRightSteerMtr_CAN_ID = 14;
      public static final int FrontRightSteerCANCoder_CAN_ID = 15;
      public static final int BackLeftDriveMtr_CAN_ID = 16;
      public static final int BackLeftSteerMtr_CAN_ID = 17;
      public static final int BackLeftSteerCANCoder_CAN_ID = 18;
      public static final int BackRightDriveMtr_CAN_ID = 19;
      public static final int BackRightSteerMtr_CAN_ID = 20;
      public static final int BackRightSteerCANCoder_CAN_ID = 21;
        /* **** End Library Components CAN ID's **** */

      /**
       * *** Start GAME Components CAN ID's ****
       */
      // ---- Other ----
      public static final int CANDLE_CAN_ID = 40;
    }

    public static final class AnalogInput_IDs {
    }

    public static final class DigitalIO_IDs {
    }

    public static final class PWM_IDs {
    }
  }
}