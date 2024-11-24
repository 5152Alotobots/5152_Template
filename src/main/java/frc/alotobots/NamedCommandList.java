package frc.alotobots;

import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDriveSubsystem;

/** Registers named commands for use with PathPlanner. */
public class NamedCommandList {

  private final SwerveDriveSubsystem driveSubsystem;

  public NamedCommandList(SwerveDriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
  }

  /** Register all named commands for PathPlanner. */
  public void registerCommands() {
    // Example: NamedCommands.registerCommand("COMMAND_NAME", CommandClass);
    // Add more commands here as needed
  }
}
