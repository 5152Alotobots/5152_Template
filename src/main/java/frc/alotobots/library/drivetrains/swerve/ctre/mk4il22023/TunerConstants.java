package frc.alotobots.library.drivetrains.swerve.ctre.mk4il22023;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.alotobots.Constants;
import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDriveSubsystem;

/**
 * Generated by the Tuner X Swerve Project Generator
 * https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
 */
public class TunerConstants {
  // Both sets of gains need to be tuned to your individual robot.
  private static final Slot0Configs steerGains =
      new Slot0Configs()
          .withKP(55)
          .withKI(0)
          .withKD(0.2)
          .withKS(0.12)
          .withKV(0.102)
          .withKA(0.015)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  private static final Slot0Configs driveGains =
      new Slot0Configs().withKP(5.5).withKI(0).withKD(0).withKS(0.13).withKV(0.099).withKA(0);

  // Pathplanner
  public static final PathConstraints PATHFINDING_CONSTRAINTS =
      new PathConstraints(5.2, 3.5, Units.degreesToRadians(540), Units.degreesToRadians(460));

  public static final PPHolonomicDriveController PP_HOLONOMIC_DRIVE_CONTROLLER =
      new PPHolonomicDriveController(
          // PID constants for translation
          new PIDConstants(2.4, 0, 0.015), // Tuned for 2022 Drive
          // PID constants for rotation
          new PIDConstants(7.8, 0, 0.015)); // Tuned for 2022 Drive

  // The closed-loop output type to use for the steer motors;
  // This affects the PID/FF gains for the steer motors
  private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  // The closed-loop output type to use for the drive motors;
  // This affects the PID/FF gains for the drive motors
  private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  // The stator current at which the wheels start to slip;
  private static final Current kSlipCurrent = Amps.of(27.16);

  // Theoretical free speed (m/s) at 12 V applied output
  public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.02);

  // Initial configs for the drive and steer motors and the CANcoder
  private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
  private static final TalonFXConfiguration steerInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(80))
                  .withStatorCurrentLimitEnable(true));
  private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
  private static final Pigeon2Configuration pigeonConfigs = null;

  // CAN bus name
  private static final CANBus kCANBus = new CANBus("", "./logs/example.hoot");

  // The frequency to run the odometry loop at
  private static final double kOdometryFrequency = 150.0; // Hz

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns
  private static final double kCoupleRatio = 3.5714285714285716;
  private static final double kDriveGearRatio = 6.746031746031747;
  private static final double kSteerGearRatio = 21.428571428571427;
  private static final Distance kWheelRadius = Inches.of(2);

  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = true;
  private static final boolean kSteerMotorInverted = true;
  private static final boolean kCanCoderInverted = false;

  // These are only used for simulation
  private static final double kSteerInertia = 0.00001;
  private static final double kDriveInertia = 0.001;
  private static final Voltage kSteerFrictionVoltage = Volts.of(0.25);
  private static final Voltage kDriveFrictionVoltage = Volts.of(0.25);

  private static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants()
          .withPigeon2Id(Constants.Robot.CanId.PIGEON_2_ID)
          .withCANBusName(kCANBus.getName())
          .withPigeon2Configs(pigeonConfigs);

  private static final SwerveModuleConstantsFactory ConstantCreator =
      new SwerveModuleConstantsFactory()
          .withDriveMotorGearRatio(kDriveGearRatio)
          .withSteerMotorGearRatio(kSteerGearRatio)
          .withWheelRadius(kWheelRadius)
          .withSlipCurrent(kSlipCurrent)
          .withSteerMotorGains(steerGains)
          .withDriveMotorGains(driveGains)
          .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
          .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
          .withSpeedAt12Volts(kSpeedAt12Volts)
          .withSteerInertia(kSteerInertia)
          .withDriveInertia(kDriveInertia)
          .withSteerFrictionVoltage(kSteerFrictionVoltage)
          .withDriveFrictionVoltage(kDriveFrictionVoltage)
          .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
          .withCouplingGearRatio(kCoupleRatio)
          .withDriveMotorInitialConfigs(driveInitialConfigs)
          .withSteerMotorInitialConfigs(steerInitialConfigs)
          .withCANcoderInitialConfigs(cancoderInitialConfigs);

  // Front Left
  private static final int kFrontLeftDriveMotorId =
      Constants.Robot.CanId.FRONT_LEFT_DRIVE_MTR_CAN_ID;
  private static final int kFrontLeftSteerMotorId =
      Constants.Robot.CanId.FRONT_LEFT_STEER_MTR_CAN_ID;
  private static final int kFrontLeftEncoderId =
      Constants.Robot.CanId.FRONT_LEFT_STEER_CAN_CODER_CAN_ID;
  private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.491943359375);
  private static final Distance kFrontLeftXPos = Inches.of(9.25);
  private static final Distance kFrontLeftYPos = Inches.of(9.25);

  // Front Right
  private static final int kFrontRightDriveMotorId =
      Constants.Robot.CanId.FRONT_RIGHT_DRIVE_MTR_CAN_ID;
  private static final int kFrontRightSteerMotorId =
      Constants.Robot.CanId.FRONT_RIGHT_STEER_MTR_CAN_ID;
  private static final int kFrontRightEncoderId =
      Constants.Robot.CanId.FRONT_RIGHT_STEER_CAN_CODER_CAN_ID;
  private static final Angle kFrontRightEncoderOffset = Rotations.of(0.1962890625);
  private static final Distance kFrontRightXPos = Inches.of(9.25);
  private static final Distance kFrontRightYPos = Inches.of(-9.25);

  // Back Left
  private static final int kBackLeftDriveMotorId = Constants.Robot.CanId.BACK_LEFT_DRIVE_MTR_CAN_ID;
  private static final int kBackLeftSteerMotorId = Constants.Robot.CanId.BACK_LEFT_STEER_MTR_CAN_ID;
  private static final int kBackLeftEncoderId =
      Constants.Robot.CanId.BACK_LEFT_STEER_CAN_CODER_CAN_ID;
  private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.468017578125);
  private static final Distance kBackLeftXPos = Inches.of(-9.25);
  private static final Distance kBackLeftYPos = Inches.of(9.25);

  // Back Right
  private static final int kBackRightDriveMotorId =
      Constants.Robot.CanId.BACK_RIGHT_DRIVE_MTR_CAN_ID;
  private static final int kBackRightSteerMotorId =
      Constants.Robot.CanId.BACK_RIGHT_STEER_MTR_CAN_ID;
  private static final int kBackRightEncoderId =
      Constants.Robot.CanId.BACK_RIGHT_STEER_CAN_CODER_CAN_ID;
  private static final Angle kBackRightEncoderOffset = Rotations.of(-0.2978515625);
  private static final Distance kBackRightXPos = Inches.of(-9.25);
  private static final Distance kBackRightYPos = Inches.of(-9.25);

  public static final SwerveModuleConstants FrontLeft =
      ConstantCreator.createModuleConstants(
          kFrontLeftSteerMotorId,
          kFrontLeftDriveMotorId,
          kFrontLeftEncoderId,
          kFrontLeftEncoderOffset,
          kFrontLeftXPos,
          kFrontLeftYPos,
          kInvertLeftSide,
          kSteerMotorInverted,
          kCanCoderInverted);
  public static final SwerveModuleConstants FrontRight =
      ConstantCreator.createModuleConstants(
          kFrontRightSteerMotorId,
          kFrontRightDriveMotorId,
          kFrontRightEncoderId,
          kFrontRightEncoderOffset,
          kFrontRightXPos,
          kFrontRightYPos,
          kInvertRightSide,
          kSteerMotorInverted,
          kCanCoderInverted);

  public static final SwerveModuleConstants BackLeft =
      ConstantCreator.createModuleConstants(
          kBackLeftSteerMotorId,
          kBackLeftDriveMotorId,
          kBackLeftEncoderId,
          kBackLeftEncoderOffset,
          kBackLeftXPos,
          kBackLeftYPos,
          kInvertLeftSide,
          kSteerMotorInverted,
          kCanCoderInverted);

  public static final SwerveModuleConstants BackRight =
      ConstantCreator.createModuleConstants(
          kBackRightSteerMotorId,
          kBackRightDriveMotorId,
          kBackRightEncoderId,
          kBackRightEncoderOffset,
          kBackRightXPos,
          kBackRightYPos,
          kInvertRightSide,
          kSteerMotorInverted,
          kCanCoderInverted);

  /**
   * Creates a SwerveDriveSubsystem instance using the constants defined in this class.
   *
   * @return The configured SwerveDriveSubsystem
   */
  public static SwerveDriveSubsystem createDrivetrain() {
    return new SwerveDriveSubsystem(
        DrivetrainConstants, kOdometryFrequency, FrontLeft, FrontRight, BackLeft, BackRight);
  }
}
