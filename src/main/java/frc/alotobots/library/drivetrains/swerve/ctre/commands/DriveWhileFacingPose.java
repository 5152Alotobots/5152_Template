package frc.alotobots.library.drivetrains.swerve.ctre.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.drivetrains.AimModule;
import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveWhileFacingPose extends Command {
    private final SwerveDriveSubsystem subSysSwerve;
    private final SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    private final DoubleSupplier velocityX;
    private final DoubleSupplier velocityY;
    private Pose2d pose;

    public DriveWhileFacingPose(
            SwerveDriveSubsystem subSysSwerve,
            DoubleSupplier velocityX,
            DoubleSupplier velocityY,
            Pose2d pose) {
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
        targetHeading = AimModule.calculateRobotHeadingAlignShooterToPose(pose, subSysSwerve.getState().Pose);
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
