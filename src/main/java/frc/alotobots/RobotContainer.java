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

import static frc.alotobots.Constants.tunerConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.swervedrive.ModulePosition;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.library.subsystems.swervedrive.commands.DefaultDrive;
import frc.alotobots.library.subsystems.swervedrive.io.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final SwerveDriveSubsystem swerveDriveSubsystem;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        swerveDriveSubsystem =
            new SwerveDriveSubsystem(
                tunerConstants,
                new GyroIOPigeon2(tunerConstants),
                new ModuleIOTalonFX(ModulePosition.FRONT_LEFT.index, tunerConstants),
                new ModuleIOTalonFX(ModulePosition.FRONT_RIGHT.index, tunerConstants),
                new ModuleIOTalonFX(ModulePosition.BACK_LEFT.index, tunerConstants),
                new ModuleIOTalonFX(ModulePosition.BACK_RIGHT.index, tunerConstants));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        swerveDriveSubsystem =
            new SwerveDriveSubsystem(
                tunerConstants,
                new GyroIO() {},
                new ModuleIOSim(ModulePosition.FRONT_LEFT.index, tunerConstants),
                new ModuleIOSim(ModulePosition.FRONT_RIGHT.index, tunerConstants),
                new ModuleIOSim(ModulePosition.BACK_LEFT.index, tunerConstants),
                new ModuleIOSim(ModulePosition.BACK_RIGHT.index, tunerConstants));
        break;

      default:
        // Replayed robot, disable IO implementations
        swerveDriveSubsystem =
            new SwerveDriveSubsystem(
                tunerConstants,
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // run configure
    configureDefaultCommands();
    configureLogicCommands();
  }

  /** Configures default commands for subsystems. */
  private void configureDefaultCommands() {

    swerveDriveSubsystem.setDefaultCommand(new DefaultDrive(swerveDriveSubsystem));
    // Add other subsystem default commands here as needed
  }

  /** Configures commands with logic (e.g., button presses). */
  private void configureLogicCommands() {

    // Add other logic-based commands here
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
