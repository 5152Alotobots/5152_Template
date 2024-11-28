package frc.alotobots.library.drivetrains.swerve.ctre;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class SwerveDrivePathPlanner {
  private final SwerveDriveSubsystem swerveDrive;
  private SendableChooser<Command> autoChooser;

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  public SwerveDrivePathPlanner(SwerveDriveSubsystem swerveDrive) {
    this.swerveDrive = swerveDrive;
    configurePathPlanner();
    registerNamedCommands();
    configureAutoChooser();
    addAutoChooserToShuffleboard();
  }

  /** Register all named commands for PathPlanner. */
  private void registerNamedCommands() {
    // Example: NamedCommands.registerCommand("COMMAND_NAME", CommandClass);
    // Add more commands here as needed
  }

  /** Configures the autonomous command chooser. */
  private void configureAutoChooser() {
    autoChooser = AutoBuilder.buildAutoChooser("Default");
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
   * Gets a command to follow a specific path.
   *
   * @param pathName The name of the path to follow.
   * @return A Command to follow the specified path.
   */
  public Command getPath(String pathName) throws IOException, ParseException {
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

    return AutoBuilder.pathfindToPose(targetPose, constraints);
  }

  private void configurePathPlanner() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> swerveDrive.getState().Pose, // Supplier of current robot pose
          swerveDrive::resetPose, // Consumer for seeding pose against auto
          () -> swerveDrive.getState().Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) ->
              swerveDrive.setControl(
                  m_pathApplyRobotSpeeds
                      .withSpeeds(speeds)
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(10, 0, 0),
              // PID constants for rotation
              new PIDConstants(7, 0, 0)),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the case
          () ->
              DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                  == DriverStation.Alliance.Red,
          swerveDrive // Subsystem for requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder" + ex, ex.getStackTrace());
    }
  }
}
