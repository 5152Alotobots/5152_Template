//   ____  _ ____ ____       _    _        ___ _____ ___  ____     ___ _____ ____
//  | ___|/ | ___|___ \    / \  | |     / _ \_   _/ _ \| __) / _ \_   _/ ___|
//  |___ \| |___ \ __) | / _ \ | |    | | | || || | | |  _ \|| | || | \___ \
//   ___) | |___) / __/   / ___ \| |__  | |_| || || |_| | |_) | |_| || |  ___) |
//  |____/|_|____/_____| /_/   \_\_____\___/ |_| \___/|____/ \___/ |_| |____/
//
//
// 2025 ALOTOBOTS FRC 5152
// Robot Code
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

  private static Translation2d getLinearVelocityFromJoysticks(
      double x, double y, SwerveDriveSubsystem swerveDriveSubsystem) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude and direction for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;
    double angleMultiplier = 1.0;

    // Apply speed modes based on triggers
    double leftTrigger = driverController.getLeftTriggerAxis();
    double rightTrigger = driverController.getRightTriggerAxis();

    if (leftTrigger > DEADBAND) {
      // Turtle mode - scale down to turtle speed
      double turtleScale =
          Constants.tunerConstants.getTurtleSpeed().in(MetersPerSecond)
              / Constants.tunerConstants.getSpeedAt12Volts().in(MetersPerSecond);
      linearMagnitude *= turtleScale;
      angleMultiplier = turtleScale;
    } else if (rightTrigger > DEADBAND) {
      // Turbo mode - scale up to turbo speed
      double turboScale =
          Constants.tunerConstants.getTurboSpeed().in(MetersPerSecond)
              / Constants.tunerConstants.getSpeedAt12Volts().in(MetersPerSecond);
      linearMagnitude *= turboScale;
      angleMultiplier = turboScale;
    }

    // Apply scaling to direction
    linearDirection = new Rotation2d(linearDirection.getRadians() * angleMultiplier);

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

  // Controllers
  private static final CommandXboxController driverController = new CommandXboxController(0);

  // Driver Controller Methods
  public static Translation2d getDriverLinearVelocity(SwerveDriveSubsystem swerveDriveSubsystem) {
    return getLinearVelocityFromJoysticks(
        driverController.getLeftX(), driverController.getLeftY(), swerveDriveSubsystem);
  }

  public static double getDriverRotation() {
    double rotation = MathUtil.applyDeadband(driverController.getRightX(), DEADBAND);
    rotation = Math.copySign(rotation * rotation, rotation);

    // Apply speed modes based on triggers
    double leftTrigger = driverController.getLeftTriggerAxis();
    double rightTrigger = driverController.getRightTriggerAxis();

    if (leftTrigger > DEADBAND) {
      // Turtle mode - scale down rotation
      rotation *=
          Constants.tunerConstants.getTurtleSpeed().in(MetersPerSecond)
              / Constants.tunerConstants.getSpeedAt12Volts().in(MetersPerSecond);
    } else if (rightTrigger > DEADBAND) {
      // Turbo mode - scale up rotation
      rotation *=
          Constants.tunerConstants.getTurboSpeed().in(MetersPerSecond)
              / Constants.tunerConstants.getSpeedAt12Volts().in(MetersPerSecond);
    }

    return rotation;
  }
}
