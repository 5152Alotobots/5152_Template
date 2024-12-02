package frc.alotobots.library.vision.photonvision.objectdetection.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDriveSubsystem;
import frc.alotobots.library.vision.photonvision.objectdetection.PhotonVisionObjectDetectionSubsystem;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class DriveFacingBestObject extends Command {
  private final PhotonVisionObjectDetectionSubsystem objectDetectionSubsystem;
  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private final DoubleSupplier velocityX;
  private final DoubleSupplier velocityY;

  private static final double DEADBAND_RADIANS = Math.toRadians(1.0);
  private final AngleKalmanFilter kalmanFilter = new AngleKalmanFilter();
  private long lastMeasurementTime = 0;
  private static final long MAX_PREDICTION_TIME_MS = 500; // Max time to predict without measurements

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

    // Tune PID for smoother response with Kalman filter
    driveFacingAngle.HeadingController = new PhoenixPIDController(0.8, 0.0, 0.1);
  }

  private Optional<Rotation2d> calculateTargetAngle() {
    // Always predict the next state
    kalmanFilter.predict();
    
    Optional<Double> rawAngle = objectDetectionSubsystem.getFieldRelativeAngle();
    if (rawAngle.isPresent()) {
      kalmanFilter.update(rawAngle.get());
      lastMeasurementTime = System.currentTimeMillis();
    } else if (System.currentTimeMillis() - lastMeasurementTime > MAX_PREDICTION_TIME_MS) {
      // Reset if we haven't seen a measurement in too long
      kalmanFilter.reset();
      return Optional.empty();
    }
    
    double filteredAngle = kalmanFilter.getAngle();
    
    // Apply deadband to the filtered angle
    if (Math.abs(filteredAngle) < DEADBAND_RADIANS) {
      return Optional.of(new Rotation2d(0.0));
    }
    
    return Optional.of(new Rotation2d(filteredAngle));
  }

  @Override
  public void execute() {
    Optional<Rotation2d> targetAngle = calculateTargetAngle();
    if (targetAngle.isEmpty()) {
      return;
    }

    swerveDriveSubsystem.setControl(
        driveFacingAngle
            .withTargetDirection(targetAngle.get())
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
    return false;
  }
}
