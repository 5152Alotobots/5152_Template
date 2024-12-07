package frc.alotobots.library.drivetrains.swerve.ctre.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDriveSubsystem;
import java.util.function.DoubleSupplier;

/**
 * Command that drives the robot while automatically facing a specified pose on the field.
 *
 * <p>This command:
 * - Takes manual drive inputs for X/Y translation
 * - Automatically rotates to face the target pose
 * - Allows temporary manual rotation override with a timeout
 * - Uses field-relative coordinates for all movements
 *
 * <p>The command requires the swerve drive subsystem to operate.
 */
public class DriveFacingPose extends Command {
  /** The target pose to face */
  private final Pose2d targetPose;


  /** The swerve drive subsystem for robot movement */
  private final SwerveDriveSubsystem swerveDriveSubsystem;

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
   * @param swerveDriveSubsystem The subsystem for controlling robot movement
   * @param velocityX Supplier for forward/backward velocity
   * @param velocityY Supplier for left/right velocity
   * @param velocityRotation Supplier for rotational velocity
   */
  public DriveFacingPose(
      Pose2d targetPose,
      SwerveDriveSubsystem swerveDriveSubsystem,
      DoubleSupplier velocityX,
      DoubleSupplier velocityY,
      DoubleSupplier velocityRotation) {
    this.targetPose = targetPose;
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.velocityX = velocityX;
    this.velocityY = velocityY;
    this.velocityRotation = velocityRotation;

    addRequirements(swerveDriveSubsystem);

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
    // Calculate angle to target pose
    var currentPose = swerveDriveSubsystem.getState().Pose;
    double dx = targetPose.getX() - currentPose.getX();
    double dy = targetPose.getY() - currentPose.getY();
    Rotation2d angleToTarget = new Rotation2d(Math.atan2(dy, dx));

    if (Math.abs(velocityRotation.getAsDouble()) < 0.1) {
      swerveDriveSubsystem.setControl(
          driveFacingAngle
              .withTargetDirection(angleToTarget)
              .withVelocityX(velocityX.getAsDouble())
              .withVelocityY(velocityY.getAsDouble()));
    } else {
      swerveDriveSubsystem.setControl(
          driveFieldCentric
              .withVelocityX(velocityX.getAsDouble())
              .withVelocityY(velocityY.getAsDouble())
              .withRotationalRate(velocityRotation.getAsDouble()));
    }

    // Start override timer when manual rotation is used
    if (Math.abs(velocityRotation.getAsDouble()) > 0.1) {
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
