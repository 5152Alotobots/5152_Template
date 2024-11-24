package frc.alotobots.library.drivetrains.swerve.ctre;

import static frc.alotobots.library.vision.photonvision.apriltag.PhotonvisionAprilTagSubsystemConstants.ONLY_USE_POSE_ESTIMATION_IN_TELEOP;
import static frc.alotobots.library.vision.photonvision.apriltag.PhotonvisionAprilTagSubsystemConstants.USE_VISION_POSE_ESTIMATION;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.alotobots.library.drivetrains.swerve.ctre.mk4il22023.TunerConstants;
import frc.alotobots.library.vision.photonvision.apriltag.PhotonvisionAprilTagSubsystem;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;

/**
 * Subsystem class for the Swerve Drive. This class extends the Phoenix SwerveDrivetrain class and
 * implements the Subsystem interface for easy integration with WPILib's command-based framework.
 */
public class SwerveDriveSubsystem extends SwerveDrivetrain implements Subsystem {
  private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
  private double lastSimTime;
  private final SwerveRequest.ApplyChassisSpeeds autoRequest =
      new SwerveRequest.ApplyChassisSpeeds();

  private PhotonvisionAprilTagSubsystem subSysPhotonvision;
  private SwerveDriveTelemetry telemetry;
  private SwerveDrivePathPlanner pathPlanner;

  // Tunable parameters
  @Getter @Setter private double maxSpeed = TunerConstants.SPEED_AT_12_VOLTS_MPS;
  @Getter @Setter private double maxAngularSpeed = Math.PI * 2;
  @Getter @Setter private double driveKP = 0.1;
  @Getter @Setter private double driveKI = 0.0;
  @Getter @Setter private double driveKD = 0.0;
  @Getter @Setter private double turnKP = 0.1;
  @Getter @Setter private double turnKI = 0.0;
  @Getter @Setter private double turnKD = 0.0;

  /**
   * Constructs a new SubSys_SwerveDrive with the given constants and modules.
   *
   * @param driveTrainConstants The constants for the swerve drivetrain.
   * @param odometryUpdateFrequency The frequency at which to update odometry.
   * @param modules The swerve module constants for each module.
   */
  public SwerveDriveSubsystem(
      SwerveDrivetrainConstants driveTrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, odometryUpdateFrequency, modules);
    initializeSubsystem();
  }

  /**
   * Constructs a new SubSys_SwerveDrive with the given constants and modules.
   *
   * @param driveTrainConstants The constants for the swerve drivetrain.
   * @param modules The swerve module constants for each module.
   */
  public SwerveDriveSubsystem(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    initializeSubsystem();
  }

  private void initializeSubsystem() {
    System.out.println("Initializing SwerveDriveSubsystem");
    this.telemetry = new SwerveDriveTelemetry(this);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    System.out.println("SwerveDriveSubsystem initialized");
  }

  private SwerveDrivePathPlanner getPathPlanner() {
    if (pathPlanner == null) {
      System.out.println("Initializing PathPlanner");
      pathPlanner = new SwerveDrivePathPlanner(this);
      System.out.println("PathPlanner initialized");
    }
    return pathPlanner;
  }

  @Override
  public void periodic() {
    // Vision estimate
    if (ONLY_USE_POSE_ESTIMATION_IN_TELEOP) {
      if (DriverStation.isTeleopEnabled()) {
        updateVisionPoseEstimate();
      }
    } else {
      updateVisionPoseEstimate();
    }

    telemetry.updateShuffleboard(this);
  }

  /**
   * Applies a SwerveRequest to the drivetrain.
   *
   * @param requestSupplier A supplier for the SwerveRequest.
   * @return A command that applies the SwerveRequest.
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /**
   * Gets the current robot chassis speeds.
   *
   * @return The current ChassisSpeeds of the robot.
   */
  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  /** Updates the robot pose with PhotonVision if tags can be seen. */
  private void updateVisionPoseEstimate() {
    if (subSysPhotonvision != null && USE_VISION_POSE_ESTIMATION) {
      Optional<Pair<Pose2d, Double>> estimatedVisionPose2d =
          subSysPhotonvision.getEstimatedVisionPose2d(this.m_odometry.getEstimatedPosition());
      estimatedVisionPose2d.ifPresent(
          pose2dDoublePair ->
              this.addVisionMeasurement(pose2dDoublePair.getFirst(), pose2dDoublePair.getSecond()));
    }
  }

  /**
   * Sets the SubSys_PhotonVision object to use for vision-based pose estimation.
   *
   * @param subSysPhotonvision The PhotonVision subsystem to use.
   */
  public void setPhotonVisionSubSys(PhotonvisionAprilTagSubsystem subSysPhotonvision) {
    this.subSysPhotonvision = subSysPhotonvision;
  }

  /** Starts the simulation thread for the swerve drive. */
  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    Notifier simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    simNotifier.startPeriodic(SIM_LOOP_PERIOD);
  }

  /**
   * Sets the auto request for PathPlanner.
   *
   * @param speeds The ChassisSpeeds to set.
   */
  public void setAutoRequest(ChassisSpeeds speeds) {
    this.setControl(autoRequest.withSpeeds(speeds));
  }

  public Pose2d getPose() {
    return this.m_odometry.getEstimatedPosition();
  }

  public double getAngularVelocity() {
    return this.m_angularVelocity.getValueAsDouble();
  }

  public double getOperatorForwardDirection() {
    return this.m_operatorForwardDirection.getDegrees();
  }

  public double getYaw() {
    return this.m_yawGetter.getValueAsDouble();
  }

  public boolean getFlipPath() {
    SwerveDrivePathPlanner planner = getPathPlanner();
    if (planner != null) {
      return planner.getFlipPath();
    } else {
      System.err.println("WARNING: PathPlanner is null in getFlipPath()");
      return false;
    }
  }

  public Translation2d[] getModuleLocations() {
    return m_moduleLocations;
  }

  /**
   * Sets the maximum speeds for the swerve drive.
   *
   * @param maxSpeed The maximum linear speed in meters per second.
   * @param maxAngularSpeed The maximum angular speed in radians per second.
   */
  public void setMaxSpeeds(double maxSpeed, double maxAngularSpeed) {
    this.maxSpeed = maxSpeed;
    this.maxAngularSpeed = maxAngularSpeed;
    // Apply the new max speeds to the swerve drive controller
    // Implementation depends on the specifics of your swerve drive controller
    System.out.println("Updating max speeds: " + maxSpeed + " m/s, " + maxAngularSpeed + " rad/s");
  }

  /**
   * Sets the PID values for the drive motors.
   *
   * @param kP The proportional gain.
   * @param kI The integral gain.
   * @param kD The derivative gain.
   */
  public void setDrivePID(double kP, double kI, double kD) {
    this.driveKP = kP;
    this.driveKI = kI;
    this.driveKD = kD;
    // Apply the new PID values to the drive motors
    for (var module : this.Modules) {
      module
          .getDriveMotor()
          .getConfigurator()
          .apply(new Slot0Configs().withKP(kP).withKI(kI).withKD(kD));
    }
  }

  /**
   * Sets the PID values for the turn motors.
   *
   * @param kP The proportional gain.
   * @param kI The integral gain.
   * @param kD The derivative gain.
   */
  public void setTurnPID(double kP, double kI, double kD) {
    this.turnKP = kP;
    this.turnKI = kI;
    this.turnKD = kD;
    // Apply the new PID values to the turn motors
    for (var module : this.Modules) {
      module
          .getSteerMotor()
          .getConfigurator()
          .apply(new Slot0Configs().withKP(kP).withKI(kI).withKD(kD));
    }
  }
}
