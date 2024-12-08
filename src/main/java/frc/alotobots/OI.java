package frc.alotobots;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OI {

    public static final double DEADBAND = 0.1;

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

    // Controllers
    private static final CommandXboxController driverController = new CommandXboxController(0);
    private static final CommandXboxController operatorController = new CommandXboxController(1);

    // Driver Controller Buttons
    public static final Trigger driverA = driverController.a();
    public static final Trigger driverB = driverController.b();
    public static final Trigger driverX = driverController.x();
    public static final Trigger driverY = driverController.y();
    public static final Trigger driverLeftBumper = driverController.leftBumper();
    public static final Trigger driverRightBumper = driverController.rightBumper();
    public static final Trigger driverBackButton = driverController.back();
    public static final Trigger driverStartButton = driverController.start();
    public static final Trigger driverLeftStickButton = driverController.leftStick();
    public static final Trigger driverRightStickButton = driverController.rightStick();

    // Driver Controller Methods
    public static Translation2d getDriverLinearVelocity() {
        return getLinearVelocityFromJoysticks(
            driverController.getLeftX(),
            -driverController.getLeftY()  // Y-axis is inverted
        );
    }

    public static double getDriverRotation() {
        return MathUtil.applyDeadband(driverController.getRightX(), DEADBAND);
    }

    public static double getDriverLeftTrigger() {
        return driverController.getLeftTriggerAxis(); // Left trigger - Variable control (e.g., slow mode)
    }

    public static double getDriverRightTrigger() {
        return driverController.getRightTriggerAxis(); // Right trigger - Variable control (e.g., shoot speed)
    }

}
