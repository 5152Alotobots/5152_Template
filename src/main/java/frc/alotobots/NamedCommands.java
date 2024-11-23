package frc.alotobots;

import com.pathplanner.lib.commands.NamedCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDriveSubsystem;

/** Registers named commands for use with PathPlanner. */
public class NamedCommands {
  
  private final SwerveDriveSubsystem driveSubsystem;

  public NamedCommands(SwerveDriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
  }

  /** Register all named commands for PathPlanner. */
  public void registerCommands() {
    // Basic drive commands
    NamedCommands.registerCommand("stop", Commands.runOnce(() -> driveSubsystem.stop()));
    NamedCommands.registerCommand("brake", Commands.runOnce(() -> driveSubsystem.setXBrake()));
    NamedCommands.registerCommand("autoBalance", driveSubsystem.autoBalanceCommand());
    
    // Add more commands here as needed
    // NamedCommands.registerCommand("intakeNote", intakeSubsystem.intakeCommand());
    // NamedCommands.registerCommand("shootNote", shooterSubsystem.shootCommand());
  }
}
