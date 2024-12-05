package frc.alotobots.library.vision.photonvision.objectdetection.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDriveSubsystem;
import frc.alotobots.library.vision.photonvision.objectdetection.PhotonVisionObjectDetectionSubsystem;
import java.util.function.DoubleSupplier;

public class DriveFacingBestObject extends Command {
  PhotonVisionObjectDetectionSubsystem objectDetectionSubsystem;
  SwerveDriveSubsystem swerveDriveSubsystem;
  DoubleSupplier velocityX;
  DoubleSupplier velocityY;
  DoubleSupplier velocityRotation;

  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.FieldCentric driveFieldCentric =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  Timer overrideTimer = new Timer();

  public DriveFacingBestObject(
      PhotonVisionObjectDetectionSubsystem objectDetectionSubsystem,
      SwerveDriveSubsystem swerveDriveSubsystem,
      DoubleSupplier velocityX,
      DoubleSupplier velocityY,
      DoubleSupplier velocityRotation) {
    this.objectDetectionSubsystem = objectDetectionSubsystem;
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.velocityX = velocityX;
    this.velocityY = velocityY;
    this.velocityRotation = velocityRotation;

    addRequirements(swerveDriveSubsystem, objectDetectionSubsystem);

    driveFacingAngle.HeadingController = new PhoenixPIDController(5.0, 0, 0.0);
  }

  @Override
  public void execute() {
    if (!objectDetectionSubsystem.getDetectedObjects().isEmpty()) {
      Rotation2d angle = objectDetectionSubsystem.getDetectedObjects().get(0).getAngle();
      swerveDriveSubsystem.setControl(
          driveFacingAngle
              .withTargetDirection(angle)
              .withVelocityX(velocityX.getAsDouble())
              .withVelocityY(velocityY.getAsDouble()));
    } else {
      swerveDriveSubsystem.setControl(
          driveFieldCentric
              .withVelocityX(velocityX.getAsDouble())
              .withVelocityY(velocityY.getAsDouble())
              .withRotationalRate(velocityRotation.getAsDouble()));
    }

    // Rotation override timeout
    if (!objectDetectionSubsystem.getDetectedObjects().isEmpty()
        && velocityRotation.getAsDouble() != 0) {
      overrideTimer.start();
    } else {
      overrideTimer.reset();
    }
  }

  private static final double OVERRIDE_TIMEOUT_SECONDS = 0.1;

  @Override
  public boolean isFinished() {
    return overrideTimer.hasElapsed(OVERRIDE_TIMEOUT_SECONDS);
  }
}
