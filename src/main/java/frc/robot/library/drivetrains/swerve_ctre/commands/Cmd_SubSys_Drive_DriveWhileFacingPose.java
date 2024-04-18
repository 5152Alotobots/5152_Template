package frc.robot.library.drivetrains.swerve_ctre.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.crescendo.HMIStation;
import frc.robot.crescendo.subsystems.shooter.expirimental.AimModule;
import frc.robot.library.drivetrains.swerve_ctre.CommandSwerveDrivetrain;

import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Drive_DriveWhileFacingSpeaker extends Command {
    private CommandSwerveDrivetrain subSysSwerve;
    private SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    private DoubleSupplier velocityX;
    private DoubleSupplier velocityY;

    public Cmd_SubSys_Drive_DriveWhileFacingSpeaker(
            CommandSwerveDrivetrain subSysSwerve,
            DoubleSupplier velocityX,
            DoubleSupplier velocityY) {
        this.subSysSwerve = subSysSwerve;
        this.velocityX = velocityX;
        this.velocityY = velocityY;
        addRequirements(subSysSwerve);
        drive.HeadingController.setPID(7,0,0);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Rotation2d targetHeading;
        if(DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
                targetHeading = AimModule.calculateRobotHeadingAlignShooterToSpeaker(subSysSwerve.getState().Pose).plus(Rotation2d.fromDegrees(195));
            } else {
                targetHeading = AimModule.calculateRobotHeadingAlignShooterToSpeaker(subSysSwerve.getState().Pose);
            }
        } else {
            targetHeading = new Rotation2d();
        }
        SmartDashboard.putNumber("Auto Aim Target Heading", targetHeading.getDegrees());
        SmartDashboard.putNumber("vx", velocityX.getAsDouble());
        SmartDashboard.putNumber("vy", velocityY.getAsDouble());
        subSysSwerve.applyRequest(() -> drive
                .withVelocityX(velocityX.getAsDouble())
                .withVelocityY(velocityY.getAsDouble())
                .withTargetDirection(targetHeading))
                .execute();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
