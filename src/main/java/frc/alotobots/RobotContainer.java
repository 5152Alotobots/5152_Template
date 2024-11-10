package frc.alotobots;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.game.HMIStation;
import frc.alotobots.library.bling.BlingSubsystem;
import frc.alotobots.library.bling.commands.DefaultSetToAllianceColor;
import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDriveSubsystem;
import frc.alotobots.library.drivetrains.swerve.ctre.mk4il22023.TunerConstants;
import frc.alotobots.library.pneumatics.PneumaticsSubsystem;
import frc.alotobots.library.vision.limelight.LimelightSubsystem;
import frc.alotobots.library.vision.photonvision.PhotonvisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final SwerveDriveSubsystem drivetrainSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final BlingSubsystem blingSubsystem;
  private final PhotonvisionSubsystem photonvisionSubsystem;
  private final PneumaticsSubsystem pneumaticsSubsystem;

  // Human-Machine Interface
  private final HMIStation hmiStation;

  // Auto Commands
  private final Auto auto;

  // Swerve drive requests
  private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric();
  private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Initialize subsystems
    drivetrainSubsystem = TunerConstants.DRIVE_TRAIN;
    blingSubsystem = new BlingSubsystem();
    limelightSubsystem = new LimelightSubsystem(blingSubsystem, drivetrainSubsystem);
    photonvisionSubsystem = new PhotonvisionSubsystem();
    pneumaticsSubsystem = new PneumaticsSubsystem();

    // Initialize HMI
    hmiStation = new HMIStation();

    // Initialize AutoCommands
    auto = new Auto(drivetrainSubsystem);

    // Configure commands and bindings
    configureDefaultCommands();
    configureLogicCommands();
    setupVision();
  }

  /** Configures default commands for subsystems. */
  private void configureDefaultCommands() {
    drivetrainSubsystem.setDefaultCommand(
          drivetrainSubsystem.applyRequest(
              () ->
                  driveFieldCentric
                      .withVelocityX(hmiStation.driveFwdAxis() * hmiStation.getDriveXYPerfMode())
                      .withVelocityY(hmiStation.driveStrAxis() * hmiStation.getDriveXYPerfMode())
                      .withRotationalRate(
                          hmiStation.driveRotAxis() * hmiStation.getDriveRotPerfMode())));
    blingSubsystem.setDefaultCommand(new DefaultSetToAllianceColor(blingSubsystem));

    // Add other subsystem default commands here as needed
  }

  /** Configures commands with logic (e.g., button presses). */
  private void configureLogicCommands() {
    hmiStation.robotCentric.whileTrue(
          drivetrainSubsystem.applyRequest(
              () ->
                  driveRobotCentric
                      .withVelocityX(hmiStation.driveFwdAxis() * hmiStation.getDriveXYPerfMode())
                      .withVelocityY(hmiStation.driveStrAxis() * hmiStation.getDriveXYPerfMode())
                      .withRotationalRate(
                          hmiStation.driveRotAxis() * hmiStation.getDriveRotPerfMode())));

    hmiStation.gyroResetButton.onTrue(
        drivetrainSubsystem.runOnce(drivetrainSubsystem::seedFieldRelative));

    // Add other logic-based commands here
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return auto.getSelectedAutoCommand();
  }

  /** Sets up the PhotonVision subsystem for the drivetrain. */
  public void setupVision() {
    drivetrainSubsystem.setPhotonVisionSubSys(photonvisionSubsystem);
  }
}
