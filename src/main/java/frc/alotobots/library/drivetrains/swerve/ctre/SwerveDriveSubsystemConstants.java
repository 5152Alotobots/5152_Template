package frc.alotobots.library.drivetrains.swerve.ctre;

import lombok.experimental.UtilityClass;

/**
 * Includes all of the constants necessary for the SwerveDrive. Does not include module or
 * pathplanner tuning
 */
public class SwerveDriveSubsystemConstants {
  public static final double DRIVE_XY_SPD_PERF_MODE_SW_FILTER_RATE = 8.0; // m/s/s
  public static final double DRIVE_ROT_SPD_PERF_MODE_SW_FILTER_RATE = 4.0; // rad/s/s
  public static final double DRIVE_TRAIN_MAX_ACCEL = 3.0; // m/s^2
  public static final double DRIVE_TRAIN_MAX_DECCEL = -2.0; // m/s^2

  @UtilityClass
  public static final class PerformanceModeDefault {
    public static final double DRIVE_TRAIN_MAX_SPD = 3.5; // m/s
    public static final double DRIVE_TRAIN_MAX_ACCELERATION = 2.0; // m/s^2
    public static final double DRIVE_TRAIN_MAX_ROT_SPD = 0.75 * 2 * Math.PI; // rad/s
    public static final double DRIVE_TRAIN_MAX_ROT_ACCELERATION = 0.75 * 2 * Math.PI; // rad/s^2
  }

  @UtilityClass
  public static final class PerformanceModeTurtle {
    public static final double DRIVE_TRAIN_MAX_SPD = 1.0; // m/s
    public static final double DRIVE_TRAIN_MAX_ACCELERATION = 0.35; // m/s^2
    public static final double DRIVE_TRAIN_MAX_ROT_SPD = 0.5 * 2 * Math.PI; // rad/s
    public static final double DRIVE_TRAIN_MAX_ROT_ACCELERATION = 0.15 * 2 * Math.PI; // rad/s^2
  }

  @UtilityClass
  public static final class PerformanceModeTurbo {
    public static final double DRIVE_TRAIN_MAX_SPD = 5.2; // m/s
    public static final double DRIVE_TRAIN_MAX_ACCELERATION = 1.00; // m/s^2
    public static final double DRIVE_TRAIN_MAX_ROT_SPD = 1.0 * 2 * Math.PI; // rad/s
    public static final double DRIVE_TRAIN_MAX_ROT_ACCELERATION = 0.75 * 2 * Math.PI; // rad/s^2
  }
}
