package frc.alotobots;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.alotobots.Constants.Robot.Calibrations;
import frc.alotobots.game.HMIStation;
import frc.alotobots.library.bling.BlingSubsystem;
import frc.alotobots.library.bling.commands.Cmd_SubSys_Bling_DefaultSetToAllianceColor;
import frc.alotobots.library.drivetrains.swerve_ctre.SwerveDriveSubsystem;
import frc.alotobots.library.drivetrains.swerve_ctre.mk4il22023.TunerConstants;
import frc.alotobots.library.vision.limelight.LimelightSubsystem;
import frc.alotobots.library.vision.photonvision.PhotonvisionSubsystem;
import lombok.Getter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Subsystems
    private final SwerveDriveSubsystem drivetrain;
    private final LimelightSubsystem limelightSubSys;
    private final BlingSubsystem blingSubSys;
    private final PhotonvisionSubsystem photonvisionSubSys;

    // Human-Machine Interface
    private final HMIStation hmiStation;

    // Swerve drive requests
    private final SwerveRequest.FieldCentric driveFieldCentric;
    private final SwerveRequest.RobotCentric driveRobotCentric;

    // Shuffleboard
    private final ShuffleboardTab driveTab;

    @Getter
    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Initialize subsystems
        drivetrain = TunerConstants.DRIVE_TRAIN;
        blingSubSys = new BlingSubsystem();
        limelightSubSys = new LimelightSubsystem(blingSubSys, drivetrain);
        photonvisionSubSys = new PhotonvisionSubsystem();

        // Initialize HMI
        hmiStation = new HMIStation();

        // Initialize drive requests
        driveFieldCentric = new SwerveRequest.FieldCentric()
                .withDeadband(Calibrations.DriveTrain.PerformanceModeDefault.DRIVE_TRAIN_MAX_SPD * 0.1)
                .withRotationalDeadband(Calibrations.DriveTrain.PerformanceModeDefault.DRIVE_TRAIN_MAX_ROT_SPD * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        driveRobotCentric = new SwerveRequest.RobotCentric()
                .withDeadband(Calibrations.DriveTrain.PerformanceModeDefault.DRIVE_TRAIN_MAX_SPD * 0.1)
                .withRotationalDeadband(Calibrations.DriveTrain.PerformanceModeDefault.DRIVE_TRAIN_MAX_ROT_SPD * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // Initialize Shuffleboard
        driveTab = Shuffleboard.getTab("Drive");

        // Initialize auto chooser
        autoChooser = drivetrain.getAutoChooser();
        driveTab.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(0, 0);

        // Configure button bindings and subsystem defaults
        configureButtonBindings();
        configureSubsystemDefaults();
        configureShuffleboard();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // Configure swerve drive controls
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> driveFieldCentric
                        .withVelocityX(hmiStation.driveFwdAxis() * hmiStation.getDriveXYPerfMode())
                        .withVelocityY(hmiStation.driveStrAxis() * hmiStation.getDriveXYPerfMode())
                        .withRotationalRate(hmiStation.driveRotAxis() * hmiStation.getDriveRotPerfMode()))
        );
        hmiStation.robotCentric.whileTrue(
                drivetrain.applyRequest(() -> driveRobotCentric
                        .withVelocityX(hmiStation.driveFwdAxis() * hmiStation.getDriveXYPerfMode())
                        .withVelocityY(hmiStation.driveStrAxis() * hmiStation.getDriveXYPerfMode())
                        .withRotationalRate(hmiStation.driveRotAxis() * hmiStation.getDriveRotPerfMode()))
        );

        // Gyro reset button
        hmiStation.gyroResetButton.onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));
    }

    /**
     * Use this method to define your subsystem default commands.
     */
    private void configureSubsystemDefaults() {
        // Set default command for LEDs
        blingSubSys.setDefaultCommand(new Cmd_SubSys_Bling_DefaultSetToAllianceColor(blingSubSys));
    }

    /**
     * Use this method to configure Shuffleboard layouts and widgets.
     */
    private void configureShuffleboard() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Sets up the PhotonVision subsystem for the drivetrain.
     */
    public void setupVision() {
        drivetrain.setPhotonVisionSubSys(photonvisionSubSys);
    }
}