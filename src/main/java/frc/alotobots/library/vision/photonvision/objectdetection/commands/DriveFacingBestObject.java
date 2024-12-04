package frc.alotobots.library.vision.photonvision.objectdetection.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDriveSubsystem;
import frc.alotobots.library.vision.photonvision.objectdetection.PhotonVisionObjectDetectionSubsystem;
import java.util.function.DoubleSupplier;

public class DriveFacingBestObject extends Command {
  PhotonVisionObjectDetectionSubsystem objectDetectionSubsystem;
  SwerveDriveSubsystem swerveDriveSubsystem;
  DoubleSupplier velocityX;
  DoubleSupplier velocityY;

  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  public DriveFacingBestObject(
      PhotonVisionObjectDetectionSubsystem objectDetectionSubsystem,
      SwerveDriveSubsystem swerveDriveSubsystem,
      DoubleSupplier velocityX,
      DoubleSupplier velocityY) {
    this.objectDetectionSubsystem = objectDetectionSubsystem;
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.velocityX = velocityX;
    this.velocityY = velocityY;

    addRequirements(swerveDriveSubsystem, objectDetectionSubsystem);

    driveFacingAngle.HeadingController = new PhoenixPIDController(5.5, 0, 0.0);
  }

  @Override
  public void execute() {
    Rotation2d angle = objectDetectionSubsystem.getDetectedObjects().get(0).getAngle();
    swerveDriveSubsystem.setControl(
        driveFacingAngle
            .withTargetDirection(angle)
            .withVelocityX(velocityX.getAsDouble())
            .withVelocityY(velocityY.getAsDouble()));
  }

  @Override
  public boolean isFinished() {
    return objectDetectionSubsystem.getDetectedObjects().isEmpty();
  }
}
