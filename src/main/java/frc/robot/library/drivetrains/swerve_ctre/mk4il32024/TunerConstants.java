package frc.robot.library.drivetrains.swerve_ctre.mk4il32024;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.library.drivetrains.swerve_ctre.SwerveDriveSubsystem;
import lombok.experimental.UtilityClass;

/**
 * This class contains tuning constants for the MK4iL3 2024 Swerve Drive.
 * It includes PID gains, gear ratios, and other configuration parameters
 * for the swerve modules and drivetrain.
 */
@UtilityClass
public class TunerConstants {

    // PID gains for the steer motors
    private static final Slot0Configs STEER_GAINS = new Slot0Configs()
            .withKP(100).withKI(0).withKD(0.2)
            .withKS(0).withKV(1.5).withKA(0);

    // PID gains for the drive motors
    private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
            .withKP(3).withKI(0).withKD(0)
            .withKS(0).withKV(0).withKA(0);

    // Output types for closed-loop control
    private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;

    // Current at which wheels start to slip
    private static final double SLIP_CURRENT_A = 60.0;

    // Theoretical free speed at 12V applied output
    public static final double SPEED_AT_12_VOLTS_MPS = 5.21;

    // Gear ratios and coupling
    private static final double COUPLE_RATIO = 3.5714285714285716;
    private static final double DRIVE_GEAR_RATIO = 6.122448979591837;
    private static final double STEER_GEAR_RATIO = 21.428571428571427;
    private static final double WHEEL_RADIUS_INCHES = 2;

    // Motor and side inversions
    private static final boolean STEER_MOTOR_REVERSED = true;
    private static final boolean INVERT_LEFT_SIDE = false;
    private static final boolean INVERT_RIGHT_SIDE = true;

    // CAN bus name
    private static final String CAN_BUS_NAME = "CTRDriveBus";

    // Simulation constants
    private static final double STEER_INERTIA = 0.00001;
    private static final double DRIVE_INERTIA = 0.001;
    private static final double STEER_FRICTION_VOLTAGE = 0.25;
    private static final double DRIVE_FRICTION_VOLTAGE = 0.25;

    // Drivetrain constants
    private static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
            .withPigeon2Id(Constants.Robot.CanId.PIGEON_2_ID)
            .withCANbusName(CAN_BUS_NAME);

    // Swerve module constants factory
    private static final SwerveModuleConstantsFactory CONSTANT_CREATOR = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            .withWheelRadius(WHEEL_RADIUS_INCHES)
            .withSlipCurrent(SLIP_CURRENT_A)
            .withSteerMotorGains(STEER_GAINS)
            .withDriveMotorGains(DRIVE_GAINS)
            .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
            .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
            .withSpeedAt12VoltsMps(SPEED_AT_12_VOLTS_MPS)
            .withSteerInertia(STEER_INERTIA)
            .withDriveInertia(DRIVE_INERTIA)
            .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
            .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(COUPLE_RATIO)
            .withSteerMotorInverted(STEER_MOTOR_REVERSED);

    // Encoder offsets for each module
    private static final double FRONT_LEFT_ENCODER_OFFSET = -0.18;
    private static final double FRONT_RIGHT_ENCODER_OFFSET = -0.47900390625;
    private static final double BACK_LEFT_ENCODER_OFFSET = 0.05615234375;
    private static final double BACK_RIGHT_ENCODER_OFFSET = 0.351318359375;

    // Module positions (in inches)
    private static final double FRONT_LEFT_X_POS_INCHES = 11.375;
    private static final double FRONT_LEFT_Y_POS_INCHES = 9.25;
    private static final double FRONT_RIGHT_X_POS_INCHES = 11.375;
    private static final double FRONT_RIGHT_Y_POS_INCHES = -9.25;
    private static final double BACK_LEFT_X_POS_INCHES = -11.375;
    private static final double BACK_LEFT_Y_POS_INCHES = 9.25;
    private static final double BACK_RIGHT_X_POS_INCHES = -11.375;
    private static final double BACK_RIGHT_Y_POS_INCHES = -9.25;

    // Swerve module constants
    private static final SwerveModuleConstants FRONT_LEFT = CONSTANT_CREATOR.createModuleConstants(
            Constants.Robot.CanId.FRONT_LEFT_STEER_MTR_CAN_ID,
            Constants.Robot.CanId.FRONT_LEFT_DRIVE_MTR_CAN_ID,
            Constants.Robot.CanId.FRONT_LEFT_STEER_CAN_CODER_CAN_ID,
            FRONT_LEFT_ENCODER_OFFSET,
            Units.inchesToMeters(FRONT_LEFT_X_POS_INCHES),
            Units.inchesToMeters(FRONT_LEFT_Y_POS_INCHES),
            INVERT_LEFT_SIDE
    );

    private static final SwerveModuleConstants FRONT_RIGHT = CONSTANT_CREATOR.createModuleConstants(
            Constants.Robot.CanId.FRONT_RIGHT_STEER_MTR_CAN_ID,
            Constants.Robot.CanId.FRONT_RIGHT_DRIVE_MTR_CAN_ID,
            Constants.Robot.CanId.FRONT_RIGHT_STEER_CAN_CODER_CAN_ID,
            FRONT_RIGHT_ENCODER_OFFSET,
            Units.inchesToMeters(FRONT_RIGHT_X_POS_INCHES),
            Units.inchesToMeters(FRONT_RIGHT_Y_POS_INCHES),
            INVERT_RIGHT_SIDE
    );

    private static final SwerveModuleConstants BACK_LEFT = CONSTANT_CREATOR.createModuleConstants(
            Constants.Robot.CanId.BACK_LEFT_STEER_MTR_CAN_ID,
            Constants.Robot.CanId.BACK_LEFT_DRIVE_MTR_CAN_ID,
            Constants.Robot.CanId.BACK_LEFT_STEER_CAN_CODER_CAN_ID,
            BACK_LEFT_ENCODER_OFFSET,
            Units.inchesToMeters(BACK_LEFT_X_POS_INCHES),
            Units.inchesToMeters(BACK_LEFT_Y_POS_INCHES),
            INVERT_LEFT_SIDE
    );

    private static final SwerveModuleConstants BACK_RIGHT = CONSTANT_CREATOR.createModuleConstants(
            Constants.Robot.CanId.BACK_RIGHT_STEER_MTR_CAN_ID,
            Constants.Robot.CanId.BACK_RIGHT_DRIVE_MTR_CAN_ID,
            Constants.Robot.CanId.BACK_RIGHT_STEER_CAN_CODER_CAN_ID,
            BACK_RIGHT_ENCODER_OFFSET,
            Units.inchesToMeters(BACK_RIGHT_X_POS_INCHES),
            Units.inchesToMeters(BACK_RIGHT_Y_POS_INCHES),
            INVERT_RIGHT_SIDE
    );

    /**
     * The swerve drivetrain subsystem, configured with the constants defined in this class.
     */
    public static final SwerveDriveSubsystem DRIVE_TRAIN = new SwerveDriveSubsystem(
            DRIVETRAIN_CONSTANTS,
            FRONT_LEFT,
            FRONT_RIGHT,
            BACK_LEFT,
            BACK_RIGHT
    );
}