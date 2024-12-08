package frc.alotobots;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OI {

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

    // Driver Controller Axes
    public static double getDriverLeftX() {
        return driverController.getLeftX();  // Left stick X axis - Used for strafing
    }

    public static double getDriverLeftY() {
        return -driverController.getLeftY(); // Left stick Y axis - Forward/backward movement
    }

    public static double getDriverRightX() {
        return driverController.getRightX(); // Right stick X axis - Rotation
    }

    public static double getDriverRightY() {
        return -driverController.getRightY(); // Right stick Y axis
    }

    public static double getDriverLeftTrigger() {
        return driverController.getLeftTriggerAxis(); // Left trigger - Variable control (e.g., slow mode)
    }

    public static double getDriverRightTrigger() {
        return driverController.getRightTriggerAxis(); // Right trigger - Variable control (e.g., shoot speed)
    }

    // Operator Controller Buttons
    public static final Trigger operatorA = operatorController.a();
    public static final Trigger operatorB = operatorController.b();
    public static final Trigger operatorX = operatorController.x();
    public static final Trigger operatorY = operatorController.y();
    public static final Trigger operatorLeftBumper = operatorController.leftBumper();
    public static final Trigger operatorRightBumper = operatorController.rightBumper();
    public static final Trigger operatorBackButton = operatorController.back();
    public static final Trigger operatorStartButton = operatorController.start();

    // Operator Controller Axes
    public static double getOperatorLeftX() {
        return operatorController.getLeftX();  // Left stick X axis - Mechanism control
    }

    public static double getOperatorLeftY() {
        return -operatorController.getLeftY(); // Left stick Y axis - Mechanism control
    }

    public static double getOperatorRightX() {
        return operatorController.getRightX(); // Right stick X axis - Mechanism control
    }

    public static double getOperatorRightY() {
        return -operatorController.getRightY(); // Right stick Y axis - Mechanism control
    }

    public static double getOperatorLeftTrigger() {
        return operatorController.getLeftTriggerAxis(); // Left trigger - Variable mechanism control
    }

    public static double getOperatorRightTrigger() {
        return operatorController.getRightTriggerAxis(); // Right trigger - Variable mechanism control
    }

    // D-Pad (POV) Methods
    public static int getDriverPOV() {
        return driverController.getHID().getPOV();
    }

    public static int getOperatorPOV() {
        return operatorController.getHID().getPOV();
    }
}
