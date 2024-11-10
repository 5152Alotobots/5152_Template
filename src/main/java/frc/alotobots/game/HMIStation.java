// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.alotobots.game;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.alotobots.Constants.Robot.Calibrations.DriveTrain;
import frc.alotobots.library.driverstation.JoystickUtilities;

/**
 * The DriverStation class represents the available inputs to the robot, providing access to
 * controllers and defining buttons for various commands.
 */
public class HMIStation {

  public static class Constants {
    // Basic drive control deadbands
    public static final double DRIVER_FWD_AXIS_DEADBAND = 0.1;
    public static final double DRIVER_STR_AXIS_DEADBAND = 0.1;
    public static final double DRIVER_ROT_AXIS_DEADBAND = 0.1;
    public static final double TRIGGER_DEADBAND = 0.3;
    
    /* Example game-specific deadbands:
     * public static final double PRIMARY_MECHANISM_AXIS_DEADBAND = 0.1;
     * public static final double SECONDARY_MECHANISM_AXIS_DEADBAND = 0.15;
     * public static final double TERTIARY_MECHANISM_AXIS_DEADBAND = 0.2;
     */
  }

  final SlewRateLimiter driveXSpdFilter =
      new SlewRateLimiter(DriveTrain.DRIVE_TRAIN_MAX_ACCEL, DriveTrain.DRIVE_TRAIN_MAX_DECCEL, 0);
  final SlewRateLimiter driveYSpdFilter =
      new SlewRateLimiter(DriveTrain.DRIVE_TRAIN_MAX_ACCEL, DriveTrain.DRIVE_TRAIN_MAX_DECCEL, 0);

  final SlewRateLimiter driveSpdPerfModeSwFilter =
      new SlewRateLimiter(DriveTrain.DRIVE_XY_SPD_PERF_MODE_SW_FILTER_RATE);
  final SlewRateLimiter driveRotPerfModeSwFilter =
      new SlewRateLimiter(DriveTrain.DRIVE_ROT_SPD_PERF_MODE_SW_FILTER_RATE);

  // **** Driver Controller ****
  private final XboxController driverController = new XboxController(0);

  /* Driver Buttons - Consider renaming these based on function rather than number
   * Example naming pattern:
   * Button 1 (A) -> primaryActionButton
   * Button 2 (B) -> secondaryActionButton
   * Button 3 (X) -> tertiaryActionButton
   * Button 4 (Y) -> quaternaryActionButton
   * etc.
   */
  public final JoystickButton driverButton1 = new JoystickButton(driverController, 1);
  public final JoystickButton driverButton2 = new JoystickButton(driverController, 2);
  public final JoystickButton driverButton3 = new JoystickButton(driverController, 3);
  public final JoystickButton gyroResetButton = new JoystickButton(driverController, 4);
  public final JoystickButton turtleModeButton = new JoystickButton(driverController, 5);
  public final JoystickButton turboModeButton = new JoystickButton(driverController, 6);

  // Driver POV
  public final POVButton driverPOVUp = new POVButton(driverController, 0);
  public final POVButton driverPOVRight = new POVButton(driverController, 90);
  public final POVButton driverPOVDown = new POVButton(driverController, 180);
  public final POVButton driverPOVLeft = new POVButton(driverController, 270);

  // Driver Axes
  /**
   * Gets the raw forward axis value for driving.
   *
   * @return The value used for driving forward. unmodified.
   */
  /**
   * Gets the raw forward axis value for driving.
   * The -1 multiplication inverts the joystick value because pushing forward
   * returns a positive value, but we want forward motion to be negative in our
   * coordinate system.
   *
   * @return The value used for driving forward with deadband applied.
   */
  public double driveFwdAxisRaw() {
    return -1
        * JoystickUtilities.joyDeadBnd(
            driverController.getRawAxis(1), Constants.DRIVER_FWD_AXIS_DEADBAND);
  }

  /**
   * Gets the Drive Forward Axis with deadband, squared and rate limited
   *
   * @return double Forward Axis with deadband, squared and rate limited
   */
  public double driveFwdAxis() {
    return driveXSpdFilter.calculate(driveFwdAxisRaw());
  }

  /**
   * Gets the strafe raw axis value for driving with deadband applied.
   *
   * @return The strafe axis value.
   */
  /**
   * Gets the strafe raw axis value for driving.
   * The -1 multiplication inverts the joystick value because pushing right
   * returns a positive value, but we want right motion to be negative in our
   * coordinate system.
   *
   * @return The strafe axis value with deadband applied.
   */
  public double driveStrAxisRaw() {
    return -1
        * JoystickUtilities.joyDeadBnd(
            driverController.getRawAxis(0), Constants.DRIVER_STR_AXIS_DEADBAND);
  }

  /**
   * Gets the Drive Strafe Axis with deadband, squared and rate limited
   *
   * @return double Strafe Axis with deadband, squared and rate limited
   */
  public double driveStrAxis() {
    return driveYSpdFilter.calculate(driveStrAxisRaw());
  }

  /**
   * Gets the rotation axis value for driving with deadband applied.
   *
   * @return The rotation axis value.
   */
  /**
   * Gets the rotation axis value for driving.
   * The -1 multiplication inverts the joystick value because pushing right
   * returns a positive value, but we want clockwise rotation to be negative
   * in our coordinate system.
   *
   * @return The rotation axis value with deadband applied.
   */
  public double driveRotAxisRaw() {
    return -1
        * JoystickUtilities.joyDeadBnd(
            driverController.getRawAxis(4), Constants.DRIVER_ROT_AXIS_DEADBAND);
  }

  /**
   * Gets the Drive Rotation Axis with deadband and squared
   *
   * @return double Rotation Axis with deadband and squared
   */
  public double driveRotAxis() {
    return JoystickUtilities.joySqrd(driveRotAxisRaw());
  }

  // Driver Trigger Axes
  public boolean robotCentricTrigger() {
    return (driverController.getRawAxis(2) > Constants.TRIGGER_DEADBAND);
  }

  public final Trigger robotCentric = new Trigger(this::robotCentricTrigger);

  /* Add game-specific trigger methods here following this pattern:
   * 
   * public boolean mechanismTrigger() {
   *   return (driverController.getRawAxis(X) > Constants.TRIGGER_DEADBAND);
   * }
   * public final Trigger mechanismTrigger = new Trigger(this::mechanismTrigger);
   */

  // **** Co-Driver Controller ****
  private final XboxController coDriverController = new XboxController(1);

  /* Co-Driver Buttons - Consider renaming these based on mechanism control
   * Example naming pattern:
   * Button 1 (A) -> primaryMechanismButton 
   * Button 2 (B) -> secondaryMechanismButton
   * Button 3 (X) -> tertiaryMechanismButton
   * Button 4 (Y) -> quaternaryMechanismButton
   * etc.
   */
  public final JoystickButton coDriverButton1 = new JoystickButton(coDriverController, 1);
  public final JoystickButton coDriverButton2 = new JoystickButton(coDriverController, 2);
  public final JoystickButton coDriverButton3 = new JoystickButton(coDriverController, 3);
  public final JoystickButton coDriverButton4 = new JoystickButton(coDriverController, 4);
  public final JoystickButton coDriverButton5 = new JoystickButton(coDriverController, 5);
  public final JoystickButton coDriverButton6 = new JoystickButton(coDriverController, 6);

  // Co-Driver POV
  public final POVButton coDriverPOVUp = new POVButton(coDriverController, 0);
  public final POVButton coDriverPOVRight = new POVButton(coDriverController, 90);
  public final POVButton coDriverPOVDown = new POVButton(coDriverController, 180);
  public final POVButton coDriverPOVLeft = new POVButton(coDriverController, 270);

  // Co-Driver Axes
  /* Add game-specific axis methods here following this pattern:
   *
   * public double mechanismAxisRaw() {
   *   return JoystickUtilities.joyDeadBnd(
   *       coDriverController.getRawAxis(X), Constants.MECHANISM_DEADBAND);
   * }
   *
   * public double mechanismAxis() {
   *   return JoystickUtilities.joyScaled(mechanismAxisRaw(), 1);
   * }
   */

  // Co Driver Trigger Axes
  public boolean coDriverLeftTrigger() {
    return (coDriverController.getRawAxis(2) > Constants.TRIGGER_DEADBAND);
  }

  public final Trigger coDriverLeftTriggerTrigger = new Trigger(this::coDriverLeftTrigger);

  public boolean coDriverRightTrigger() {
    return (coDriverController.getRawAxis(3) > Constants.TRIGGER_DEADBAND);
  }

  public final Trigger coDriverRightTriggerTrigger = new Trigger(this::coDriverRightTrigger);

  // Aux Driver Controller
  private final XboxController auxdriverController = new XboxController(2);

  public final JoystickButton auxButton1 = new JoystickButton(auxdriverController, 1);
  public final JoystickButton auxButton2 = new JoystickButton(auxdriverController, 2);

  // public final JoystickButton auxButton3 = new JoystickButton(auxdriverController, 3);
  // public final JoystickButton auxButton4 = new JoystickButton(auxdriverController, 4);
  // public final JoystickButton auxButton5 = new JoystickButton(auxdriverController, 5);
  // public final JoystickButton auxButton6 = new JoystickButton(auxdriverController, 6);
  // Button Box

  /**
   * @return XY mode
   */
  public double getDriveXYPerfMode() {
    double xySpeed = DriveTrain.PerformanceModeDefault.DRIVE_TRAIN_MAX_SPD;
    if (turtleModeButton.getAsBoolean()) {
      xySpeed = DriveTrain.PerformanceModeTurtle.DRIVE_TRAIN_MAX_SPD;
    } else if (turboModeButton.getAsBoolean()) {
      xySpeed = DriveTrain.PerformanceModeTurbo.DRIVE_TRAIN_MAX_SPD;
    }
    return driveSpdPerfModeSwFilter.calculate(xySpeed);
  }

  /**
   * @return Rot mode
   */
  public double getDriveRotPerfMode() {
    double rotSpeed = DriveTrain.PerformanceModeDefault.DRIVE_TRAIN_MAX_ROT_SPD;
    if (turtleModeButton.getAsBoolean()) {
      rotSpeed = DriveTrain.PerformanceModeTurtle.DRIVE_TRAIN_MAX_ROT_SPD;
    } else if (turboModeButton.getAsBoolean()) {
      rotSpeed = DriveTrain.PerformanceModeTurbo.DRIVE_TRAIN_MAX_ROT_SPD;
    }
    return driveRotPerfModeSwFilter.calculate(rotSpeed);
  }

  // ---- Alliance Color
  public void alliancePosition() {
    DriverStation.getAlliance();
  }
}
