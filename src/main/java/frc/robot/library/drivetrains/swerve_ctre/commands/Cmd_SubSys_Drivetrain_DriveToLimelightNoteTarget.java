// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library.drivetrains.swerve_ctre.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.library.drivetrains.swerve_ctre.CommandSwerveDrivetrain;

public class Cmd_SubSys_Drivetrain_DriveToLimelightNoteTarget extends Command {
  /** Creates a new Cmd_SubSys_Drivetrain_DriveToLimelightTarget. */
  private CommandSwerveDrivetrain drivetrain;
  private BooleanSupplier noteDetected;
  private DoubleSupplier tx;

  private RobotCentric drive = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
 
  // Create a PID controller
  // velocity and acceleration constraints.
  private final ProfiledPIDController rotController =
      new ProfiledPIDController(3.0, 0.0, 0.0,
      new TrapezoidProfile.Constraints(10, 10));

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1, 0.0, 0.0); 
  
  public Cmd_SubSys_Drivetrain_DriveToLimelightNoteTarget(CommandSwerveDrivetrain drivetrain, BooleanSupplier noteDetected, DoubleSupplier tx) {
    this.drivetrain = drivetrain;
    this.noteDetected = noteDetected;
    this.tx = tx;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotController.setTolerance(0.01);
    rotController.setGoal(0);
    //rotController.reset(tx,drivetrain.getState().speeds.omegaRadiansPerSecond);
    //rotController.reset(tx);
    //double rotCmd = rotController.calculate(tx.getAsDouble(), 0);//+feedforward.calculate(drivetrain.getState().speeds.omegaRadiansPerSecond);

    drivetrain.applyRequest(() -> 
      drive.withVelocityX(drivetrain.getCurrentRobotChassisSpeeds().vxMetersPerSecond)
        .withVelocityY(0)
        .withRotationalRate(0)).execute();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotCmd = rotController.calculate(Units.degreesToRadians(tx.getAsDouble()), 0); //+feedforward.calculate(drivetrain.getState().speeds.omegaRadiansPerSecond);
    double ffCmd = feedforward.calculate(rotController.getSetpoint().velocity);
    SmartDashboard.putNumber("LimelightCmdTx", Units.degreesToRadians(tx.getAsDouble()));
    SmartDashboard.putNumber("LimelightRotCmd", rotCmd);
    SmartDashboard.putNumber("LimelightFFCmd", ffCmd);
    drivetrain.applyRequest(() -> 
      drive.withVelocityX(1)
        .withVelocityY(0)
        .withRotationalRate(rotCmd)).execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() -> 
      drive.withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0)).execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
