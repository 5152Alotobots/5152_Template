package frc.alotobots;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDriveSubsystem;

/** This class contains methods to set up and manage autonomous commands for the robot. */
public class Auto {

  private final SwerveDriveSubsystem drivetrainSubsystem;
  private SendableChooser<Command> autoChooser;

  /**
   * Constructs an AutoCommands object with the necessary subsystems.
   *
   * @param drivetrainSubsystem The SwerveDriveSubsystem
   */
  public Auto(SwerveDriveSubsystem drivetrainSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.autoChooser = new SendableChooser<>();
    
    // Register named commands before configuring auto chooser
    NamedCommands namedCommands = new NamedCommands(drivetrainSubsystem);
    namedCommands.registerCommands();
    
    configureAutoChooser();
    addAutoChooserToShuffleboard();
  }

  /** Configures the autonomous command chooser. */
  private void configureAutoChooser() {
    autoChooser = getAutoChooser();
    // Add more auto options here
  }

  /** Adds the auto chooser to the Shuffleboard. */
  private void addAutoChooserToShuffleboard() {
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
    driveTab.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(2, 4);
  }

  /**
   * Gets the selected autonomous command.
   *
   * @return The selected autonomous command
   */
  public Command getSelectedAutoCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Gets an autonomous command for the specified path.
   *
   * @param pathName The name of the path to follow.
   * @return A Command to run the specified autonomous path.
   */
  public Command getAutoPath(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  /**
   * Gets a SendableChooser for selecting autonomous routines.
   *
   * @return A SendableChooser containing available autonomous routines.
   */
  public SendableChooser<Command> getAutoChooser() {
    return AutoBuilder.buildAutoChooser("DEFAULT_COMMAND_NAME");
  }

  /**
   * Gets a command to follow a specific path.
   *
   * @param pathName The name of the path to follow.
   * @return A Command to follow the specified path.
   */
  public Command getPath(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    return AutoBuilder.followPath(path);
  }

  /**
   * Gets a command to pathfind to a target pose.
   *
   * @param targetPose The target pose to pathfind to.
   * @return A Command to pathfind to the specified pose.
   */
  public Command getPathFinderCommand(Pose2d targetPose) {
    PathConstraints constraints =
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    return AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters
        );
  }
}
