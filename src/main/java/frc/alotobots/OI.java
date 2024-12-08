package frc.alotobots;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class OI {

    public static final double DEADBAND = 0.1;

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Apply speed modes based on triggers
        double leftTrigger = driverController.getLeftTriggerAxis();
        double rightTrigger = driverController.getRightTriggerAxis();

        if (leftTrigger > DEADBAND) {
            // Turtle mode - scale down to turtle speed
            linearMagnitude *= Constants.tunerConstants.getTurtleSpeed().in(MetersPerSecond) / 
                              Constants.tunerConstants.getSpeedAt12Volts().in(MetersPerSecond);
        } else if (rightTrigger > DEADBAND) {
            // Turbo mode - scale up to turbo speed
            linearMagnitude *= Constants.tunerConstants.getTurboSpeed().in(MetersPerSecond) / 
                              Constants.tunerConstants.getSpeedAt12Volts().in(MetersPerSecond);
        }

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

    // Controllers
    private static final CommandXboxController driverController = new CommandXboxController(0);

    // Driver Controller Methods
    public static Translation2d getDriverLinearVelocity() {
        return getLinearVelocityFromJoysticks(
            driverController.getLeftX(),
            driverController.getLeftY()
        );
    }

    public static double getDriverRotation() {
        return MathUtil.applyDeadband(driverController.getRightX(), DEADBAND);
    }


}
