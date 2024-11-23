package frc.alotobots;

import com.pathplanner.lib.commands.NamedCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
    // Example named commands - replace with your actual commands
    com.pathplanner.lib.commands.NamedCommand.registerCommand(
        "stop", Commands.runOnce(() -> driveSubsystem.stop()));
        
    com.pathplanner.lib.commands.NamedCommand.registerCommand(
        "brake", Commands.runOnce(() -> driveSubsystem.setXBrake()));
  }
}
