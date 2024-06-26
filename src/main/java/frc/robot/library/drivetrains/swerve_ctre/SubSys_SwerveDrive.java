package frc.robot.library.drivetrains.swerve_ctre;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.library.drivetrains.swerve_ctre.mk4il32024.TunerConstants_MK4iL3_2024;
import frc.robot.library.vision.photonvision.SubSys_Photonvision;

import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import static frc.robot.library.vision.photonvision.SubSys_Photonvision_Constants.ONLY_USE_POSE_ESTIMATION_IN_TELEOP;
import static frc.robot.library.vision.photonvision.SubSys_Photonvision_Constants.USE_VISION_POSE_ESTIMATION;

/**
 * Subsystem class for the Swerve Drive, extending CTRE's SwerveDrivetrain and implementing WPILib's Subsystem.
 * This class manages the swerve drive functionality, including autonomous path following and vision-based pose estimation.
 */
public class SubSys_SwerveDrive extends SwerveDrivetrain implements Subsystem {
    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
    private double lastSimTime;
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private boolean flipPath = false;

    private final Field2d field = new Field2d();

    private SubSys_Photonvision subSysPhotonvision;

    /**
     * Constructor for the SubSys_SwerveDrive with a specified odometry update frequency.
     *
     * @param driveTrainConstants    The constants for the swerve drivetrain
     * @param odometryUpdateFrequency The frequency at which to update odometry
     * @param modules                The constants for each swerve module
     */
    public SubSys_SwerveDrive(SwerveDrivetrainConstants driveTrainConstants, double odometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, odometryUpdateFrequency, modules);
        initializeSubsystem();
    }

    /**
     * Constructor for the SubSys_SwerveDrive without specifying an odometry update frequency.
     *
     * @param driveTrainConstants The constants for the swerve drivetrain
     * @param modules             The constants for each swerve module
     */
    public SubSys_SwerveDrive(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        initializeSubsystem();
    }

    /**
     * Initializes the subsystem by configuring PathPlanner, simulation (if applicable), and logging.
     */
    private void initializeSubsystem() {
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        initLog();
    }

    @Override
    public void periodic() {
        // Update Field2d with the estimated robot pose
        field.setRobotPose(this.m_odometry.getEstimatedPosition());

        // Update vision-based pose estimate if enabled
        if (ONLY_USE_POSE_ESTIMATION_IN_TELEOP) {
            if (DriverStation.isTeleopEnabled()) {
                updateVisionPoseEstimate();
            }
        } else {
            updateVisionPoseEstimate();
        }
    }

    /**
     * Updates the robot pose with PhotonVision data if tags can be seen.
     */
    public void updateVisionPoseEstimate() {
        if (subSysPhotonvision != null && USE_VISION_POSE_ESTIMATION) {
            Optional<Pair<Pose2d, Double>> estimatedVisionPose2d = subSysPhotonvision.getEstimatedVisionPose2d(this.m_odometry.getEstimatedPosition());
            estimatedVisionPose2d.ifPresent(pose2dDoublePair -> this.addVisionMeasurement(pose2dDoublePair.getFirst(), pose2dDoublePair.getSecond()));
        }
    }

    /**
     * Sets the SubSys_PhotonVision object to use for vision-based pose estimation.
     *
     * @param subSysPhotonvision The PhotonVision subsystem to use
     */
    public void setPhotonVisionSubSys(SubSys_Photonvision subSysPhotonvision) {
        this.subSysPhotonvision = subSysPhotonvision;
    }

    /**
     * Creates a command that applies a SwerveRequest to the drivetrain.
     *
     * @param requestSupplier A supplier for the SwerveRequest
     * @return A Command that applies the SwerveRequest
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Gets the current robot chassis speeds.
     *
     * @return The current ChassisSpeeds of the robot
     */
    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    /**
     * Starts the simulation thread for the swerve drive.
     */
    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        Notifier simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(SIM_LOOP_PERIOD);
    }

    /**
     * Gets an autonomous command for a specified path.
     *
     * @param pathName The name of the path to follow
     * @return A Command to run the specified autonomous path
     */
    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    /**
     * Gets a SendableChooser for selecting autonomous routines.
     *
     * @return A SendableChooser containing available autonomous routines
     */
    public SendableChooser<Command> getAutoChooser() {
        return AutoBuilder.buildAutoChooser("DEFAULT_COMMAND_NAME");
    }

    /**
     * Gets a command to follow a specific path.
     *
     * @param pathName The name of the path to follow
     * @return A Command to follow the specified path
     */
    public Command getPath(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return AutoBuilder.followPath(path);
    }

    /**
     * Creates a command to pathfind to a target pose.
     *
     * @param targetPose The target Pose2d to reach
     * @return A Command to pathfind to the target pose
     */
    public Command getPathFinderCommand(Pose2d targetPose) {
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        return AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0  // Rotation delay distance in meters
        );
    }

    /**
     * Configures PathPlanner for use with the swerve drive.
     */
    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        flipPath = DriverStation.getAlliance().isPresent() &&
                DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose,
                this::seedFieldRelative,
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),
                new HolonomicPathFollowerConfig(
                        new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        TunerConstants_MK4iL3_2024.SPEED_AT_12_VOLTS_MPS,
                        driveBaseRadius,
                        new ReplanningConfig()
                ),
                () -> flipPath,
                this
        );
    }

    /**
     * Initializes logging and Shuffleboard displays for the swerve drive.
     */
    private void initLog() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drive");
        ShuffleboardLayout poseList = tab
                .getLayout("Pose", BuiltInLayouts.kList)
                .withSize(2, 3)
                .withProperties(Map.of("Label position", "HIDDEN"));

        tab.add("DriveAngularVelocity", this.m_angularVelocity.getValueAsDouble());
        tab.add("DriveOprForwardDirection", this.m_operatorForwardDirection.getDegrees());

        poseList.add("PoseX", this.m_odometry.getEstimatedPosition().getX());
        poseList.add("PoseY", this.m_odometry.getEstimatedPosition().getY());
        poseList.add("Rotation", this.m_yawGetter.getValueAsDouble());

        poseList.add("Field", field);

        tab.add("FlipPath", this.flipPath);
    }
}