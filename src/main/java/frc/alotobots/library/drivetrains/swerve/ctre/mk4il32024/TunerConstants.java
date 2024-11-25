package frc.alotobots.library.drivetrains.swerve.ctre.mk4il32024;

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
import edu.wpi.first.units.measure.*;
import frc.alotobots.Constants;
import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDriveSubsystem;
import lombok.experimental.UtilityClass;

/**
 * This class contains tuning constants for the MK4iL3 2024 Swerve Drive. It includes PID gains,
 * gear ratios, and other configuration parameters for the swerve modules and drivetrain.
 */
@UtilityClass
public class TunerConstants {

  private static final Slot0Configs steerGains =
      new Slot0Configs()
          .withKP(100)
          .withKI(0)
          .withKD(0.2)
          .withKS(0)
          .withKV(1.5)
          .withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  private static final Slot0Configs driveGains =
      new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0).withKV(0);

  private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  private static final Current kSlipCurrent = Amps.of(60.0);
  public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.21);

  private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
  private static final TalonFXConfiguration steerInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(60))
                  .withStatorCurrentLimitEnable(true));
  private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
  private static final Pigeon2Configuration pigeonConfigs = null;

  private static final CANBus kCANBus = new CANBus("CTRDriveBus", "./logs/example.hoot");
  private static final double kOdometryFrequency = 250.0; // Hz

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns
  private static final double kCoupleRatio = 3.5714285714285716;
  private static final double kDriveGearRatio = 6.122448979591837;
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
  private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.18);
  private static final Distance kFrontLeftXPos = Inches.of(11.375);
  private static final Distance kFrontLeftYPos = Inches.of(9.25);

  // Front Right
  private static final int kFrontRightDriveMotorId =
      Constants.Robot.CanId.FRONT_RIGHT_DRIVE_MTR_CAN_ID;
  private static final int kFrontRightSteerMotorId =
      Constants.Robot.CanId.FRONT_RIGHT_STEER_MTR_CAN_ID;
  private static final int kFrontRightEncoderId =
      Constants.Robot.CanId.FRONT_RIGHT_STEER_CAN_CODER_CAN_ID;
  private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.47900390625);
  private static final Distance kFrontRightXPos = Inches.of(11.375);
  private static final Distance kFrontRightYPos = Inches.of(-9.25);

  // Back Left
  private static final int kBackLeftDriveMotorId = Constants.Robot.CanId.BACK_LEFT_DRIVE_MTR_CAN_ID;
  private static final int kBackLeftSteerMotorId = Constants.Robot.CanId.BACK_LEFT_STEER_MTR_CAN_ID;
  private static final int kBackLeftEncoderId =
      Constants.Robot.CanId.BACK_LEFT_STEER_CAN_CODER_CAN_ID;
  private static final Angle kBackLeftEncoderOffset = Rotations.of(0.05615234375);
  private static final Distance kBackLeftXPos = Inches.of(-11.375);
  private static final Distance kBackLeftYPos = Inches.of(9.25);

  // Back Right
  private static final int kBackRightDriveMotorId =
      Constants.Robot.CanId.BACK_RIGHT_DRIVE_MTR_CAN_ID;
  private static final int kBackRightSteerMotorId =
      Constants.Robot.CanId.BACK_RIGHT_STEER_MTR_CAN_ID;
  private static final int kBackRightEncoderId =
      Constants.Robot.CanId.BACK_RIGHT_STEER_CAN_CODER_CAN_ID;
  private static final Angle kBackRightEncoderOffset = Rotations.of(0.351318359375);
  private static final Distance kBackRightXPos = Inches.of(-11.375);
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
