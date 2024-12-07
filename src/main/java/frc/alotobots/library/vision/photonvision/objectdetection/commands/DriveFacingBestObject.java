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

/**
 * Command that drives the robot while automatically facing the best detected object. Uses
 * PhotonVision object detection to identify targets and adjusts robot orientation accordingly.
 *
 * <p>This command: - Takes manual drive inputs for X/Y translation - Automatically rotates to face
 * the highest-confidence detected object - Falls back to manual rotation control when no objects
 * are detected - Allows temporary manual rotation override with a timeout
 *
 * <p>The command requires both the vision and drive subsystems to operate.
 */
public class DriveFacingBestObject extends Command {
  /** The subsystem handling object detection via PhotonVision */
  private final PhotonVisionObjectDetectionSubsystem objectDetectionSubsystem;

  /** The swerve drive subsystem for robot movement */
  private final SwerveDriveSubsystem swerveDriveSubsystem;

  /** The specific game element type to target */
  private final GameElement targetGameElement;

  /** Supplier for X velocity (forward/backward) */
  private final DoubleSupplier velocityX;

  /** Supplier for Y velocity (left/right) */
  private final DoubleSupplier velocityY;

  /** Supplier for rotational velocity */
  private final DoubleSupplier velocityRotation;

  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.FieldCentric driveFieldCentric =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  Timer overrideTimer = new Timer();

  /**
   * Creates a new DriveFacingBestObject command.
   *
   * @param objectDetectionSubsystem The subsystem for detecting objects
   * @param swerveDriveSubsystem The subsystem for controlling robot movement
   * @param velocityX Supplier for forward/backward velocity
   * @param velocityY Supplier for left/right velocity
   * @param velocityRotation Supplier for rotational velocity
   */
  public DriveFacingBestObject(
      PhotonVisionObjectDetectionSubsystem objectDetectionSubsystem,
      SwerveDriveSubsystem swerveDriveSubsystem,
      GameElement targetGameElement,
      DoubleSupplier velocityX,
      DoubleSupplier velocityY,
      DoubleSupplier velocityRotation) {
    this.objectDetectionSubsystem = objectDetectionSubsystem;
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.targetGameElement = targetGameElement;
    this.velocityX = velocityX;
    this.velocityY = velocityY;
    this.velocityRotation = velocityRotation;

    addRequirements(swerveDriveSubsystem, objectDetectionSubsystem);

    driveFacingAngle.HeadingController = new PhoenixPIDController(5.0, 0, 0.0);
  }

  /**
   * Called repeatedly when this Command is scheduled to run. Controls robot movement while facing
   * detected objects.
   *
   * <p>The control flow: 1. If objects are detected: - Uses field-centric drive with automatic
   * rotation to face best object 2. If no objects detected: - Falls back to standard field-centric
   * drive with manual rotation 3. If manual rotation override is active: - Starts timeout timer for
   * returning to automatic facing
   */
  @Override
  public void execute() {
    var detectedObjects = objectDetectionSubsystem.getDetectedObjects().stream()
        .filter(obj -> obj.getGameElement().getName().equals(targetGameElement.getName()))
        .toList();

    if (!detectedObjects.isEmpty()) {
      Rotation2d angle = detectedObjects.get(0).getAngle();
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
    if (!detectedObjects.isEmpty()
        && velocityRotation.getAsDouble() != 0) {
      overrideTimer.start();
    } else {
      overrideTimer.reset();
    }
  }

  private static final double OVERRIDE_TIMEOUT_SECONDS = 0.1;

  /**
   * Returns true when the command should end. Ends when rotation override timeout has elapsed.
   *
   * @return true if the command should end
   */
  @Override
  public boolean isFinished() {
    return overrideTimer.hasElapsed(OVERRIDE_TIMEOUT_SECONDS);
  }
}
