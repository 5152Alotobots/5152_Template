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

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.Units;
import frc.alotobots.Constants;
import frc.alotobots.library.drivetrains.swerve.ctre.SwerveDriveSubsystem;
import lombok.experimental.UtilityClass;

/**
 * This class contains tuning constants for the MK4iL2 2023 Swerve Drive. It includes PID gains,
 * gear ratios, and other configuration parameters for the swerve modules and drivetrain.
 */
@UtilityClass
public class TunerConstants {

  // CRITICAL:
  private static final double ODOMETRY_UPDATE_FREQUENCY = 100; // In Hz. Default 100

  // PID gains for the steer motors
  private static final Slot0Configs STEER_GAINS =
      new Slot0Configs().withKP(100).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);

  // PID gains for the drive motors
  private static final Slot0Configs DRIVE_GAINS =
      new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

  // Output types for closed-loop control
  private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
  private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;

  // Current at which wheels start to slip
  private static final double SLIP_CURRENT_A = 300.0;

  // Theoretical free speed at 12V applied output
  public static final double SPEED_AT_12_VOLTS_MPS = 4.73;

  // Gear ratios and coupling
  private static final double COUPLE_RATIO = 3.5714285714285716;
  private static final double DRIVE_GEAR_RATIO = 6.746031746031747;
  private static final double STEER_GEAR_RATIO = 21.428571428571427;
  private static final double WHEEL_RADIUS_INCHES = 2;

  // Motor and side inversions
  private static final boolean STEER_MOTOR_INVERTED = true; // We are assuming that you are not mixing and matching mk4is with regular ones?
  private static final boolean INVERT_LEFT_SIDE = false;
  private static final boolean INVERT_RIGHT_SIDE = true;

  // CAN bus name
  private static final String CANBUS_NAME = "";

  // Simulation constants
  private static final double STEER_INERTIA = 0.00001;
  private static final double DRIVE_INERTIA = 0.001;
  private static final double STEER_FRICTION_VOLTAGE = 0.25;
  private static final double DRIVE_FRICTION_VOLTAGE = 0.25;

  // Drivetrain constants
  private static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS =
      new SwerveDrivetrainConstants()
          .withPigeon2Id(Constants.Robot.CanId.PIGEON_2_ID)
          .withCANBusName(CANBUS_NAME);

  // Swerve module constants factory
  private static final SwerveModuleConstantsFactory CONSTANT_CREATOR =
      new SwerveModuleConstantsFactory()
          .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
          .withSteerMotorGearRatio(STEER_GEAR_RATIO)
          .withWheelRadius(WHEEL_RADIUS_INCHES)
          .withSlipCurrent(SLIP_CURRENT_A)
          .withSteerMotorGains(STEER_GAINS)
          .withDriveMotorGains(DRIVE_GAINS)
          .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
          .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
          .withSpeedAt12Volts(SPEED_AT_12_VOLTS_MPS)
          .withSteerInertia(STEER_INERTIA)
          .withDriveInertia(DRIVE_INERTIA)
          .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
          .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE)
          .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
          .withCouplingGearRatio(COUPLE_RATIO);

  // Encoder offsets for each module
  private static final double FRONT_LEFT_ENCODER_OFFSET = 0.491943359375;
  private static final double FRONT_RIGHT_ENCODER_OFFSET = 0.1962890625;
  private static final double BACK_LEFT_ENCODER_OFFSET = -0.468017578125;
  private static final double BACK_RIGHT_ENCODER_OFFSET = -0.2978515625;

  // Module positions (in inches)
  private static final double FRONT_LEFT_X_POS_INCHES = 9.25;
  private static final double FRONT_LEFT_Y_POS_INCHES = 9.25;
  private static final double FRONT_RIGHT_X_POS_INCHES = 9.25;
  private static final double FRONT_RIGHT_Y_POS_INCHES = -9.25;
  private static final double BACK_LEFT_X_POS_INCHES = -9.25;
  private static final double BACK_LEFT_Y_POS_INCHES = 9.25;
  private static final double BACK_RIGHT_X_POS_INCHES = -9.25;
  private static final double BACK_RIGHT_Y_POS_INCHES = -9.25;

  // Swerve module constants
  private static final SwerveModuleConstants FRONT_LEFT =
      CONSTANT_CREATOR.createModuleConstants(
          Constants.Robot.CanId.FRONT_LEFT_STEER_MTR_CAN_ID,
          Constants.Robot.CanId.FRONT_LEFT_DRIVE_MTR_CAN_ID,
          Constants.Robot.CanId.FRONT_LEFT_STEER_CAN_CODER_CAN_ID,
          FRONT_LEFT_ENCODER_OFFSET,
          Meters.convertFrom(FRONT_LEFT_X_POS_INCHES, Inches),
          Meters.convertFrom(FRONT_LEFT_Y_POS_INCHES, Inches),
          INVERT_LEFT_SIDE,
              STEER_MOTOR_INVERTED);

  private static final SwerveModuleConstants FRONT_RIGHT =
      CONSTANT_CREATOR.createModuleConstants(
          Constants.Robot.CanId.FRONT_RIGHT_STEER_MTR_CAN_ID,
          Constants.Robot.CanId.FRONT_RIGHT_DRIVE_MTR_CAN_ID,
          Constants.Robot.CanId.FRONT_RIGHT_STEER_CAN_CODER_CAN_ID,
          FRONT_RIGHT_ENCODER_OFFSET,
          Meters.convertFrom(FRONT_RIGHT_X_POS_INCHES, Inches),
          Meters.convertFrom(FRONT_RIGHT_Y_POS_INCHES, Inches),
          INVERT_RIGHT_SIDE,
              STEER_MOTOR_INVERTED);

  private static final SwerveModuleConstants BACK_LEFT =
      CONSTANT_CREATOR.createModuleConstants(
          Constants.Robot.CanId.BACK_LEFT_STEER_MTR_CAN_ID,
          Constants.Robot.CanId.BACK_LEFT_DRIVE_MTR_CAN_ID,
          Constants.Robot.CanId.BACK_LEFT_STEER_CAN_CODER_CAN_ID,
          BACK_LEFT_ENCODER_OFFSET,
          Meters.convertFrom(BACK_LEFT_X_POS_INCHES, Inches),
          Meters.convertFrom(BACK_LEFT_Y_POS_INCHES, Inches),
          INVERT_LEFT_SIDE,
              STEER_MOTOR_INVERTED);

  private static final SwerveModuleConstants BACK_RIGHT =
      CONSTANT_CREATOR.createModuleConstants(
          Constants.Robot.CanId.BACK_RIGHT_STEER_MTR_CAN_ID,
          Constants.Robot.CanId.BACK_RIGHT_DRIVE_MTR_CAN_ID,
          Constants.Robot.CanId.BACK_RIGHT_STEER_CAN_CODER_CAN_ID,
          BACK_RIGHT_ENCODER_OFFSET,
          Meters.convertFrom(BACK_RIGHT_X_POS_INCHES, Inches),
          Meters.convertFrom(BACK_RIGHT_Y_POS_INCHES, Inches),
          INVERT_RIGHT_SIDE,
              STEER_MOTOR_INVERTED);

  /** The swerve drivetrain subsystem, configured with the constants defined in this class. */
  public static final SwerveDriveSubsystem DRIVE_TRAIN =
      new SwerveDriveSubsystem(
          DRIVETRAIN_CONSTANTS, ODOMETRY_UPDATE_FREQUENCY, FRONT_RIGHT, FRONT_LEFT, BACK_LEFT, BACK_RIGHT);
}
