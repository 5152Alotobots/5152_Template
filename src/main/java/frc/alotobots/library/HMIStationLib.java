package frc.alotobots.library;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.alotobots.Constants;
import frc.alotobots.library.driverstation.JoystickUtilities;

public abstract class HMIStationLib {
    final SlewRateLimiter driveFwdSpdFilter = new SlewRateLimiter(Constants.Robot.Calibrations.DriveTrain.DRIVE_TRAIN_MAX_ACCEL, Constants.Robot.Calibrations.DriveTrain.DRIVE_TRAIN_MAX_DECCEL,0);
    final SlewRateLimiter driveStrSpdFilter = new SlewRateLimiter(Constants.Robot.Calibrations.DriveTrain.DRIVE_TRAIN_MAX_ACCEL, Constants.Robot.Calibrations.DriveTrain.DRIVE_TRAIN_MAX_DECCEL,0);

  final SlewRateLimiter driveSpdPerfModeFilter = new SlewRateLimiter(Constants.Robot.Calibrations.DriveTrain.DRIVE_XY_SPD_PERF_MODE_SW_FILTER_RATE);
  final SlewRateLimiter driveRotPerfModeFilter = new SlewRateLimiter(Constants.Robot.Calibrations.DriveTrain.DRIVE_ROT_SPD_PERF_MODE_SW_FILTER_RATE);

  protected abstract JoystickButton getTurtleModeButton();
  protected abstract JoystickButton getTurboModeButton();
  protected abstract JoystickButton getGyroResetButton();
  protected abstract JoystickButton getRobotCentricButton();

  /**
   * Gets the forward drive axis (not rate limited).
   *
   * @return The forward speed, typically between -1.0 and 1.0.
   */
  protected abstract double getDriveFwdAxis();

  /**
   * Gets the strafe drive axis (not rate limited).
   *
   * @return The strafe speed, typically between -1.0 and 1.0.
   */
  protected abstract double getDriveStrAxis();

  /**
   * Gets the rotation drive axis (not rate limited).
   *
   * @return The rotation speed, typically between -1.0 and 1.0.
     */
    protected abstract double getDriveRotAxis();


    public double getDriveFwd() {
      return -1 * driveFwdSpdFilter.calculate(getDriveFwdAxis());
    }

    public double getDriveStr() {
      return -1 * driveStrSpdFilter.calculate(getDriveStrAxis());
    }

    public double getDriveRot() {
      return -1 * JoystickUtilities.joyDeadBndSqrd(getDriveRotAxis(), 0.2);
    }

    /**
     * Gets the current drive performance mode (speed multiplier).
     *
     * @return The current drive performance mode for XY movement.
     */
    public double getDriveXYPerfMode() {
      double xySpeed = Constants.Robot.Calibrations.DriveTrain.PerformanceModeDefault.DRIVE_TRAIN_MAX_SPD;
      if (getTurtleModeButton().getAsBoolean()) {
        xySpeed = Constants.Robot.Calibrations.DriveTrain.PerformanceModeTurtle.DRIVE_TRAIN_MAX_SPD;
      } else if (getTurboModeButton().getAsBoolean()) {
        xySpeed = Constants.Robot.Calibrations.DriveTrain.PerformanceModeTurbo.DRIVE_TRAIN_MAX_SPD;
      }
      return driveSpdPerfModeFilter.calculate(xySpeed);
    }

    ;

    /**
     * Gets the current rotation performance mode (speed multiplier).
     *
     * @return The current drive performance mode for rotation.
     */
    public double getDriveRotPerfMode() {
      double rotSpeed = Constants.Robot.Calibrations.DriveTrain.PerformanceModeDefault.DRIVE_TRAIN_MAX_ROT_SPD;
      if (getTurtleModeButton().getAsBoolean()) {
        rotSpeed = Constants.Robot.Calibrations.DriveTrain.PerformanceModeTurtle.DRIVE_TRAIN_MAX_ROT_SPD;
      } else if (getTurboModeButton().getAsBoolean()) {
        rotSpeed = Constants.Robot.Calibrations.DriveTrain.PerformanceModeTurbo.DRIVE_TRAIN_MAX_ROT_SPD;
      }
      return driveRotPerfModeFilter.calculate(rotSpeed);
    }
}
