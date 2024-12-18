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

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;

public class OI {
  public static final double DEADBAND = 0.1;

  // Controllers
  private static final CommandXboxController driverController = new CommandXboxController(0);

  /**
   * Calculates the linear velocity vector based on joystick inputs and drive mode.
   *
   * @param x X-axis joystick input (-1.0 to 1.0)
   * @param y Y-axis joystick input (-1.0 to 1.0)
   * @param swerveDriveSubsystem The swerve drive subsystem for field-relative calculations
   * @return Translation2d representing the desired linear velocity vector
   */
  private static Translation2d getLinearVelocityFromJoysticks(
      double x, double y, SwerveDriveSubsystem swerveDriveSubsystem) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(-y, -x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;
    double angleMultiplier = 1.0;

    // Start with nominal speed as the default
    double speedScale =
        Constants.tunerConstants.getNominalSpeed().in(MetersPerSecond)
            / Constants.tunerConstants.getSpeedAt12Volts().in(MetersPerSecond);

    // Apply speed modes based on triggers
    double leftTrigger = driverController.getLeftTriggerAxis();
    double rightTrigger = driverController.getRightTriggerAxis();

    if (leftTrigger > DEADBAND) {
      speedScale =
          Constants.tunerConstants.getTurtleSpeed().in(MetersPerSecond)
              / Constants.tunerConstants.getSpeedAt12Volts().in(MetersPerSecond);
    } else if (rightTrigger > DEADBAND) {
      speedScale =
          Constants.tunerConstants.getTurboSpeed().in(MetersPerSecond)
              / Constants.tunerConstants.getSpeedAt12Volts().in(MetersPerSecond);
    }

    // Apply speed scaling to magnitude
    linearMagnitude *= speedScale;

    // Convert to field relative based on alliance
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    Rotation2d fieldRotation =
        isFlipped
            ? swerveDriveSubsystem.getRotation().plus(new Rotation2d(Math.PI))
            : swerveDriveSubsystem.getRotation();

    // Return new linear velocity with field relative conversion
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, fieldRotation))
        .getTranslation();
  }

  /**
   * Calculates the rotational velocity based on joystick input and drive mode.
   *
   * @param rotation Raw rotation input from joystick (-1.0 to 1.0)
   * @return double representing the desired rotational velocity
   */
  private static double getRotationalVelocityFromJoystick(double rotation) {
    // Apply deadband and square inputs for more precise control
    rotation = MathUtil.applyDeadband(-rotation, DEADBAND);
    rotation = Math.copySign(rotation * rotation, rotation);

    // Start with nominal speed as the default
    double speedScale =
        Constants.tunerConstants.getNominalSpeed().in(MetersPerSecond)
            / Constants.tunerConstants.getSpeedAt12Volts().in(MetersPerSecond);

    // Apply speed modes based on triggers
    double leftTrigger = driverController.getLeftTriggerAxis();
    double rightTrigger = driverController.getRightTriggerAxis();

    if (leftTrigger > DEADBAND) {
      // Turtle mode - scale down rotation
      speedScale =
          Constants.tunerConstants.getTurtleSpeed().in(MetersPerSecond)
              / Constants.tunerConstants.getSpeedAt12Volts().in(MetersPerSecond);
    } else if (rightTrigger > DEADBAND) {
      // Turbo mode - scale up rotation
      speedScale =
          Constants.tunerConstants.getTurboSpeed().in(MetersPerSecond)
              / Constants.tunerConstants.getSpeedAt12Volts().in(MetersPerSecond);
    }

    return rotation * speedScale;
  }

  /**
   * Gets the desired linear velocity vector from the driver's left joystick input. Applies
   * deadband, scaling, and field-relative transformations.
   *
   * @param swerveDriveSubsystem The swerve drive subsystem for field-relative calculations
   * @return Translation2d representing the desired linear velocity vector
   */
  public static Translation2d getDriverLinearVelocity(SwerveDriveSubsystem swerveDriveSubsystem) {
    // Remember, the controller coordinate system is inverse to that of WPILIB. WPILIB uses X
    // forwards and XboxController uses X Lateral
    return getLinearVelocityFromJoysticks(
        driverController.getLeftY(), driverController.getLeftX(), swerveDriveSubsystem);
  }

  /**
   * Gets the desired rotational velocity from the driver's right joystick input. Applies deadband,
   * scaling, and drive mode modifications.
   *
   * @return double representing the desired rotational velocity
   */
  public static double getDriverRotation() {
    return getRotationalVelocityFromJoystick(driverController.getRightX());
  }
}
