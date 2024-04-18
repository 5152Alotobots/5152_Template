/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Robot.Calibrations;
import frc.robot.game.HMIStation;
import frc.robot.library.bling.SubSys_Bling;
import frc.robot.library.bling.commands.Cmd_SubSys_Bling_DefaultSetToAllianceColor;
import frc.robot.library.drivetrains.swerve_ctre.CommandSwerveDrivetrain;
import frc.robot.library.drivetrains.swerve_ctre.Telemetry;
import frc.robot.library.vision.limelight.SubSys_Limelight;
import frc.robot.library.vision.photonvision.SubSys_Photonvision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("UnusedAssignment")
public class RobotContainer {

  public final SendableChooser<Command> autoChooser;

   public RobotContainer() {

    // Instance Variables
    final CommandSwerveDrivetrain drivetrain;
    final SwerveRequest.RobotCentric driveRC;
    final SwerveRequest.FieldCentric drive;
    final Telemetry logger;
    final SubSys_Limelight limelightSubSys;
    final HMIStation hmiStation;
    final SubSys_Bling blingSubSys;
    final SubSys_Photonvision photonvisionSubSys;


    /* Subsystems */
       drivetrain = frc.robot.library.drivetrains.swerve_ctre.mk4il32024.TunerConstants_MK4iL3_2024.DriveTrain;
            
       drive = new SwerveRequest.FieldCentric()
               .withDeadband(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd * 0.1)
               .withRotationalDeadband(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxRotSpd * 0.1) // Add a 10% deadband
               .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
            
       driveRC = new SwerveRequest.RobotCentric()
               .withDeadband(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd * 0.1)
               .withRotationalDeadband(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxRotSpd * 0.1) // Add a 10% deadband
               .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

       logger = new Telemetry(Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd);

       blingSubSys = new SubSys_Bling();

       limelightSubSys = new SubSys_Limelight(blingSubSys, drivetrain);

       hmiStation = new HMIStation();

       photonvisionSubSys = new SubSys_Photonvision("camFront");

       // ---- Auto ----
       // Register Named Commands for PathPlanner
       // Syntax: NamedCommands.registerCommand("COMMAND_NAME", new COMMAND_OBJECT);

       // Auto Chooser
       autoChooser = drivetrain.getAutoChooser();
       SmartDashboard.putData("Auto Chooser", autoChooser);

       // Configure the button bindings
       configure(
               drivetrain,
               drive,
               driveRC,
               logger,
               hmiStation,
               blingSubSys,
               photonvisionSubSys
       );
  }


/**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}. Use this method to define your
   * button->command mappings. Buttons can be created by instantiating a {@link GenericHID} or one
   * of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configure(
    CommandSwerveDrivetrain drivetrain,
    SwerveRequest.FieldCentric drive,
    SwerveRequest.RobotCentric driveRC,
    Telemetry logger,
    HMIStation hmiStation,
    SubSys_Bling subSysBling,
    SubSys_Photonvision subSysPhotonvision) {

      // Vision
      drivetrain.setPhotonVisionSubSys(subSysPhotonvision);

      // ---- Drive Subsystem ----
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(hmiStation.driveFwdAxis() * hmiStation.getDriveXYPerfMode()) // Drive forward with negative Y (forward)
                .withVelocityY(hmiStation.driveStrAxis() * hmiStation.getDriveXYPerfMode()) // Drive left with negative X (left)
                .withRotationalRate(hmiStation.driveRotAxis() * hmiStation.getDriveRotPerfMode())) // Drive counterclockwise with negative X (left)
        );
      hmiStation.robotCentric.whileTrue(
              drivetrain.applyRequest(() ->
                      driveRC.withVelocityX(hmiStation.driveFwdAxis() * hmiStation.getDriveXYPerfMode()) // Drive forward with negative Y (forward).withVelocityY(hmiStation.driveStrAxis() * hmiStation.getDriveXYPerfMode()) // Drive left with negative X (left)
                       .withRotationalRate(hmiStation.driveRotAxis() * hmiStation.getDriveRotPerfMode())) // Drive counterclockwise with negative X (left)
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

      // OTHER SUBSYSTEMS BELOW!

    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}