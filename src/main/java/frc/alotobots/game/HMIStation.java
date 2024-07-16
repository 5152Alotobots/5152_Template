package frc.alotobots.game;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.alotobots.library.HMIStationLib;
import frc.alotobots.library.driverstation.JoystickUtilities;
import lombok.Getter;

public class HMIStation extends HMIStationLib {
  private final XboxController driverController = new XboxController(0);
  private final XboxController coDriverController = new XboxController(1);

  // Declare Robot Buttons
  @Getter
  private final JoystickButton intakeOut;
  @Getter
  private final JoystickButton intakeIn;

  //Make Getters for
  @Getter
  private final JoystickButton turtleModeButton;
  @Getter
  private final JoystickButton turboModeButton;
  @Getter
  private final JoystickButton gyroResetButton;
  @Getter
  private final JoystickButton robotCentricButton;

  public HMIStation() {
    // Initialize buttons
    turtleModeButton = new JoystickButton(driverController, 5);
    robotCentricButton = new JoystickButton(driverController, 2);
    turboModeButton = new JoystickButton(driverController, 6);
    gyroResetButton = new JoystickButton(driverController, 4);

    intakeOut = new JoystickButton(coDriverController, 5);
    intakeIn = new JoystickButton(coDriverController, 6);
  }

  @Override
  protected double getDriveFwdAxis() {
    return -driverController.getRawAxis(1);  // Invert Y-axis
  }

  @Override
  protected double getDriveStrAxis() {
    return -driverController.getRawAxis(0);  // Invert X-axis
  }

  @Override
  protected double getDriveRotAxis() {
    return -driverController.getRawAxis(4);  // Invert rotation axis
  }

  public double shooterArmAxis() {
    return JoystickUtilities.joyDeadBndScaled(-coDriverController.getRawAxis(5), .5, 1);
  }

  public double intakeArmAxis() {
    return coDriverController.getRawAxis(1);
  }

}