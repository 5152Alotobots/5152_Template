package frc.alotobots;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.game.HMIStation;
import frc.alotobots.library.bling.BlingSubsystem;
import frc.alotobots.library.bling.commands.DefaultSetToAllianceColor;
import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDriveSubsystem;
import frc.alotobots.library.drivetrains.swerve.ctre.mk4il22023.TunerConstants;
import frc.alotobots.library.pneumatics.PneumaticsSubsystem;
import frc.alotobots.library.vision.photonvision.apriltag.PhotonvisionAprilTagSubsystem;
import frc.alotobots.library.vision.photonvision.objectdetection.PhotonVisionObjectDetectionSubsystem;
import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDrivePathPlanner;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final SwerveDriveSubsystem drivetrainSubsystem;
  private final BlingSubsystem blingSubsystem;
  private final PhotonvisionAprilTagSubsystem photonvisionAprilTagSubsystem;
  private final PhotonVisionObjectDetectionSubsystem photonvisionObjectDetectionSubsystem;
  private final PneumaticsSubsystem pneumaticsSubsystem;

  // Human-Machine Interface
  private final HMIStation hmiStation;

  // Path Planning and Auto
  private final SwerveDrivePathPlanner pathPlanner;

  // Swerve drive requests
  private final SwerveRequest.FieldCentric driveFieldCentric =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Initialize subsystems
    // ━━━━━━━━━━━━━━━━━━━━ [ Drive ] ━━━━━━━━━━━━━━━━━━━━
    drivetrainSubsystem = TunerConstants.createDrivetrain(); // Currently using: 2023 L2 Version

    // ━━━━━━━━━━━━━━━━━━━━ [ Bling ] ━━━━━━━━━━━━━━━━━━━━
    blingSubsystem = new BlingSubsystem();

    // ━━━━━━━━━━━━━━━━━━━━ [ April Tag ] ━━━━━━━━━━━━━━━━━━━━
    photonvisionAprilTagSubsystem = new PhotonvisionAprilTagSubsystem(drivetrainSubsystem);

    // ━━━━━━━━━━━━━━━━━━━━ [ Object Detection ] ━━━━━━━━━━━━━━━━━━━━
    photonvisionObjectDetectionSubsystem =
        new PhotonVisionObjectDetectionSubsystem(drivetrainSubsystem);

    // ━━━━━━━━━━━━━━━━━━━━ [ Pneumatics ] ━━━━━━━━━━━━━━━━━━━━
    pneumaticsSubsystem = new PneumaticsSubsystem();

    // Initialize HMI
    hmiStation = new HMIStation();

    // Initialize Path Planning and Auto
    pathPlanner = new SwerveDrivePathPlanner(drivetrainSubsystem);

    // Configure commands and bindings
    configureDefaultCommands();
    configureLogicCommands();
  }

  /** Configures default commands for subsystems. */
  private void configureDefaultCommands() {
    // ━━━━━━━━━━━━━━━━━━━━ [ Drive ] ━━━━━━━━━━━━━━━━━━━━
    drivetrainSubsystem.setDefaultCommand(
        drivetrainSubsystem.applyRequest(
            () ->
                driveFieldCentric
                    .withVelocityX(hmiStation.driveFwdAxis() * hmiStation.getDriveXYPerfMode())
                    .withVelocityY(hmiStation.driveStrAxis() * hmiStation.getDriveXYPerfMode())
                    .withRotationalRate(
                        hmiStation.driveRotAxis() * hmiStation.getDriveRotPerfMode())));

    // ━━━━━━━━━━━━━━━━━━━━ [ Bling ] ━━━━━━━━━━━━━━━━━━━━
    blingSubsystem.setDefaultCommand(new DefaultSetToAllianceColor(blingSubsystem));

    // Add other subsystem default commands here as needed
  }

  /** Configures commands with logic (e.g., button presses). */
  private void configureLogicCommands() {
    hmiStation.gyroResetButton.onTrue(
        drivetrainSubsystem.runOnce(drivetrainSubsystem::seedFieldCentric));

    // Add other logic-based commands here
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return pathPlanner.getSelectedAutoCommand();
  }
}
