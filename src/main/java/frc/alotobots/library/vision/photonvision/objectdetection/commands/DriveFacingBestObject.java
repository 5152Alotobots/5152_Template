package frc.alotobots.library.vision.photonvision.objectdetection.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
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
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
          .withSteerRequestType(SwerveModule.SteerRequestType.Position);

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
  }

  @Override
  public void execute() {
    if (objectDetectionSubsystem.getDetectedObjects().isEmpty()) {
      System.out.println("No objects detected");
      // return;
    }
    // System.out.println("Objects detected");
    // System.out.println(objectDetectionSubsystem.getDetectedObjects().get(0).getAngle());
    // Rotation2d targetRotation = Rotation2d.fromDegrees(180);
    swerveDriveSubsystem.setControl(
        driveFacingAngle
            .withTargetDirection(Rotation2d.fromDegrees(90))
            .withVelocityX(velocityX.getAsDouble())
            .withVelocityY(velocityY.getAsDouble()));
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("Interrupted!!");
    }
  }

  @Override
  public boolean isFinished() {
    return false; // objectDetectionSubsystem.getDetectedObjects().isEmpty();
  }
}
