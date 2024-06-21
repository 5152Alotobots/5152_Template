package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Robot.Calibrations;
import frc.robot.game.HMIStation;
import frc.robot.library.bling.SubSys_Bling;
import frc.robot.library.bling.commands.Cmd_SubSys_Bling_DefaultSetToAllianceColor;
import frc.robot.library.drivetrains.swerve_ctre.SubSys_SwerveDrive;
import frc.robot.library.drivetrains.swerve_ctre.Telemetry;
import frc.robot.library.drivetrains.swerve_ctre.mk4il22023.TunerConstants_MK4iL2_2023;
import frc.robot.library.vision.limelight.SubSys_Limelight;
import frc.robot.library.vision.photonvision.SubSys_Photonvision;
import org.photonvision.PhotonCamera;

public class RobotContainer {

    public final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        // Instance Variables
        final SubSys_SwerveDrive drivetrain;
        final SwerveRequest.RobotCentric driveRC;
        final SwerveRequest.FieldCentric drive;
        final Telemetry logger;
        final SubSys_Limelight limelightSubSys;
        final HMIStation hmiStation;
        final SubSys_Bling blingSubSys;
        final SubSys_Photonvision photonvisionSubSys;

        // Shuffleboard Tabs
        final ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

        /* Subsystems */
        drivetrain = TunerConstants_MK4iL2_2023.DriveTrain;

        drive = new SwerveRequest.FieldCentric()
                .withDeadband(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd * 0.1)
                .withRotationalDeadband(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxRotSpd * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        driveRC = new SwerveRequest.RobotCentric()
                .withDeadband(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd * 0.1)
                .withRotationalDeadband(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxRotSpd * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        logger = new Telemetry(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd);

        blingSubSys = new SubSys_Bling();

        limelightSubSys = new SubSys_Limelight(blingSubSys, drivetrain);

        hmiStation = new HMIStation();

        photonvisionSubSys = new SubSys_Photonvision();

        // Auto Chooser
        autoChooser = drivetrain.getAutoChooser();
        driveTab.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(0, 0);

        // Configure the button bindings
        configure(
                drivetrain,
                drive,
                driveRC,
                logger,
                hmiStation,
                blingSubSys,
                photonvisionSubSys,
                driveTab
        );
    }

    private void configure(
            SubSys_SwerveDrive drivetrain,
            SwerveRequest.FieldCentric drive,
            SwerveRequest.RobotCentric driveRC,
            Telemetry logger,
            HMIStation hmiStation,
            SubSys_Bling subSysBling,
            SubSys_Photonvision subSysPhotonvision,
            ShuffleboardTab driveTab) {

        // Vision
        drivetrain.setPhotonVisionSubSys(subSysPhotonvision);

        // ---- Drive Subsystem ----
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() ->
                        drive.withVelocityX(hmiStation.driveFwdAxis() * hmiStation.getDriveXYPerfMode())
                                .withVelocityY(hmiStation.driveStrAxis() * hmiStation.getDriveXYPerfMode())
                                .withRotationalRate(hmiStation.driveRotAxis() * hmiStation.getDriveRotPerfMode()))
        );
        hmiStation.robotCentric.whileTrue(
                drivetrain.applyRequest(() ->
                        driveRC.withVelocityX(hmiStation.driveFwdAxis() * hmiStation.getDriveXYPerfMode())
                                .withVelocityY(hmiStation.driveStrAxis() * hmiStation.getDriveXYPerfMode())
                                .withRotationalRate(hmiStation.driveRotAxis() * hmiStation.getDriveRotPerfMode()))
        );
        // ---- Sim Drive Subsystem ----
        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        // ---- Telemetry ----
        drivetrain.registerTelemetry(logger::telemeterize);

        // ---- Gyro Reset ----
        hmiStation.gyroResetButton.onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

        // ---- LEDs ----
        subSysBling.setDefaultCommand(new Cmd_SubSys_Bling_DefaultSetToAllianceColor(subSysBling)); // Default

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}