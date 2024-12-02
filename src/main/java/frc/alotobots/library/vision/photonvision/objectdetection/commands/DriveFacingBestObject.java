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

  // Smoothing and control parameters
  private static final double SMOOTHING_FACTOR = 0.8; // Very strong smoothing
  private static final int MOVING_AVERAGE_WINDOW = 10; // Larger window
  private static final double DEADBAND_RADIANS = Math.toRadians(2.0); // 2 degree deadband
  private static final double MAX_ANGLE_CHANGE_RATE = Math.toRadians(10); // More conservative rate limit

  // Smoothing state variables
  private double lastSmoothedAngle = 0.0;
  private double lastOutputAngle = 0.0;
  private boolean hasValidMeasurement = false;
  private final double[] angleBuffer = new double[MOVING_AVERAGE_WINDOW];
  private int bufferIndex = 0;
  private int validSamples = 0;

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

    driveFacingAngle.HeadingController = new PhoenixPIDController(0.5, 0.01, 0.05);
  }

  private Optional<Double> applyExponentialSmoothing(double rawAngle) {
    if (!hasValidMeasurement) {
      lastSmoothedAngle = rawAngle;
      hasValidMeasurement = true;
    } else {
      lastSmoothedAngle = SMOOTHING_FACTOR * rawAngle + (1 - SMOOTHING_FACTOR) * lastSmoothedAngle;
    }
    return Optional.of(lastSmoothedAngle);
  }

  private double calculateMovingAverage(double smoothedAngle) {
    angleBuffer[bufferIndex] = smoothedAngle;
    bufferIndex = (bufferIndex + 1) % MOVING_AVERAGE_WINDOW;
    validSamples = Math.min(validSamples + 1, MOVING_AVERAGE_WINDOW);

    double sum = 0;
    for (int i = 0; i < validSamples; i++) {
      sum += angleBuffer[i];
    }
    return sum / validSamples;
  }

  private Optional<Double> applyDeadbandAndScaling(double angle) {
    if (Math.abs(angle) < DEADBAND_RADIANS) {
      return Optional.of(0.0);
    }

    // Simple rate limiting
    double angleChange = angle - lastOutputAngle;
    if (Math.abs(angleChange) > MAX_ANGLE_CHANGE_RATE) {
      angleChange = Math.copySign(MAX_ANGLE_CHANGE_RATE, angleChange);
    }

    lastOutputAngle = lastOutputAngle + angleChange;
    return Optional.of(lastOutputAngle);
  }

  private Optional<Rotation2d> calculateTargetAngle() {
    Optional<Double> rawAngle = objectDetectionSubsystem.getFieldRelativeAngle();
    if (rawAngle.isEmpty()) {
      hasValidMeasurement = false;
      return Optional.empty();
    }

    return applyExponentialSmoothing(rawAngle.get())
        .map(this::calculateMovingAverage)
        .flatMap(this::applyDeadbandAndScaling)
        .map(Rotation2d::new);
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
