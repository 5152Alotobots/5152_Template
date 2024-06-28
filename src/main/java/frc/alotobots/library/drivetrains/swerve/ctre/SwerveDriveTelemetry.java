package frc.alotobots.library.drivetrains.swerve.ctre;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.math.geometry.Pose2d;
import frc.alotobots.Constants;

import java.util.Map;

/**
 * Handles telemetry for the Swerve Drive subsystem.
 */
public class SwerveDriveTelemetry {
    private final ShuffleboardTab driveTab;
    private final ShuffleboardLayout poseList;
    private final ShuffleboardLayout tunableParamsList;
    private final Field2d field;

    // Pose entries
    private final GenericEntry poseXEntry;
    private final GenericEntry poseYEntry;
    private final GenericEntry rotationEntry;

    // Tunable parameters
    private SimpleWidget maxSpeedWidget;
    private SimpleWidget maxAngularSpeedWidget;
    private SimpleWidget driveKPWidget;
    private SimpleWidget driveKIWidget;
    private SimpleWidget driveKDWidget;
    private SimpleWidget turnKPWidget;
    private SimpleWidget turnKIWidget;
    private SimpleWidget turnKDWidget;

    /**
     * Constructs a new SwerveDriveTelemetry object.
     *
     * @param swerveDrive The SwerveDriveSubsystem to provide telemetry for.
     */
    public SwerveDriveTelemetry(SwerveDriveSubsystem swerveDrive) {
        this.driveTab = Shuffleboard.getTab("Drive");
        this.field = new Field2d();
        this.poseList = initializePoseList(swerveDrive);
        this.tunableParamsList = initializeTunableParameters(swerveDrive);

        // Initialize pose entries
        this.poseXEntry = poseList.add("Pose X", 0.0).getEntry();
        this.poseYEntry = poseList.add("Pose Y", 0.0).getEntry();
        this.rotationEntry = poseList.add("Rotation", 0.0).getEntry();

        initializeField();
        initializeOtherWidgets(swerveDrive);
    }

    /**
     * Initializes the pose list in Shuffleboard.
     *
     * @param swerveDrive The SwerveDriveSubsystem to provide telemetry for.
     * @return The initialized ShuffleboardLayout for pose information.
     */
    private ShuffleboardLayout initializePoseList(SwerveDriveSubsystem swerveDrive) {
        return driveTab
                .getLayout("Pose", BuiltInLayouts.kList)
                .withSize(2, 3)
                .withPosition(0, 0)
                .withProperties(Map.of("Label position", "LEFT"));
    }

    /**
     * Initializes the field widget in Shuffleboard.
     */
    private void initializeField() {
        driveTab.add("Field", field)
                .withPosition(2, 0)
                .withSize(6, 4);
    }

    /**
     * Initializes other widgets in Shuffleboard.
     *
     * @param swerveDrive The SwerveDriveSubsystem to provide telemetry for.
     */
    private void initializeOtherWidgets(SwerveDriveSubsystem swerveDrive) {
        driveTab.addBoolean("Tuning Mode", () -> Constants.Robot.TUNE_MODE)
                .withPosition(0, 3)
                .withSize(2, 1);
        driveTab.addBoolean("Flip Path", swerveDrive::getFlipPath)
                .withPosition(0, 4)
                .withSize(2, 1);
    }

    /**
     * Initializes tunable parameters in Shuffleboard.
     *
     * @param swerveDrive The SwerveDriveSubsystem to provide telemetry for.
     * @return The initialized ShuffleboardLayout for tunable parameters.
     */
    private ShuffleboardLayout initializeTunableParameters(SwerveDriveSubsystem swerveDrive) {
        ShuffleboardLayout tunableParamsList = driveTab
                .getLayout("Tunable Parameters", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(8, 0)
                .withProperties(Map.of("Label position", "LEFT"));

        maxSpeedWidget = tunableParamsList.add("Max Speed (m/s)", swerveDrive.getMaxSpeed());
        maxAngularSpeedWidget = tunableParamsList.add("Max Angular Speed (rad/s)", swerveDrive.getMaxAngularSpeed());
        driveKPWidget = tunableParamsList.add("Drive kP", swerveDrive.getDriveKP());
        driveKIWidget = tunableParamsList.add("Drive kI", swerveDrive.getDriveKI());
        driveKDWidget = tunableParamsList.add("Drive kD", swerveDrive.getDriveKD());
        turnKPWidget = tunableParamsList.add("Turn kP", swerveDrive.getTurnKP());
        turnKIWidget = tunableParamsList.add("Turn kI", swerveDrive.getTurnKI());
        turnKDWidget = tunableParamsList.add("Turn kD", swerveDrive.getTurnKD());

        return tunableParamsList;
    }

    /**
     * Updates the Shuffleboard with the latest telemetry data.
     *
     * @param swerveDrive The SwerveDriveSubsystem to update telemetry for.
     */
    public void updateShuffleboard(SwerveDriveSubsystem swerveDrive) {
        Pose2d currentPose = swerveDrive.getPose();

        // Update pose entries with truncated values
        poseXEntry.setDouble(truncate(currentPose.getX(), 3));
        poseYEntry.setDouble(truncate(currentPose.getY(), 3));
        rotationEntry.setDouble(truncate(currentPose.getRotation().getDegrees(), 3));

        // Update field widget
        field.setRobotPose(currentPose);

        if (Constants.Robot.TUNE_MODE) {
            updateTunableParameters(swerveDrive);
        }
    }

    /**
     * Updates tunable parameters in Shuffleboard.
     *
     * @param swerveDrive The SwerveDriveSubsystem to update parameters for.
     */
    private void updateTunableParameters(SwerveDriveSubsystem swerveDrive) {
        double newMaxSpeed = maxSpeedWidget.getEntry().getDouble(swerveDrive.getMaxSpeed());
        double newMaxAngularSpeed = maxAngularSpeedWidget.getEntry().getDouble(swerveDrive.getMaxAngularSpeed());

        if (newMaxSpeed != swerveDrive.getMaxSpeed() || newMaxAngularSpeed != swerveDrive.getMaxAngularSpeed()) {
            System.out.println("Updating SwerveMaxSpeed");
            swerveDrive.setMaxSpeeds(newMaxSpeed, newMaxAngularSpeed);
        }

        double newDriveKP = driveKPWidget.getEntry().getDouble(swerveDrive.getDriveKP());
        double newDriveKI = driveKIWidget.getEntry().getDouble(swerveDrive.getDriveKI());
        double newDriveKD = driveKDWidget.getEntry().getDouble(swerveDrive.getDriveKD());

        if (newDriveKP != swerveDrive.getDriveKP() || newDriveKI != swerveDrive.getDriveKI() || newDriveKD != swerveDrive.getDriveKD()) {
            System.out.println("Updating SwerveDrivePID");
            swerveDrive.setDrivePID(newDriveKP, newDriveKI, newDriveKD);
        }

        double newTurnKP = turnKPWidget.getEntry().getDouble(swerveDrive.getTurnKP());
        double newTurnKI = turnKIWidget.getEntry().getDouble(swerveDrive.getTurnKI());
        double newTurnKD = turnKDWidget.getEntry().getDouble(swerveDrive.getTurnKD());

        if (newTurnKP != swerveDrive.getTurnKP() || newTurnKI != swerveDrive.getTurnKI() || newTurnKD != swerveDrive.getTurnKD()) {
            System.out.println("Updating SwerveTurnPID");
            swerveDrive.setTurnPID(newTurnKP, newTurnKI, newTurnKD);
        }
    }

    /**
     * Truncates a double value to a specified number of decimal places.
     *
     * @param value The value to truncate.
     * @param places The number of decimal places to keep.
     * @return The truncated value.
     */
    private double truncate(double value, int places) {
        double scale = Math.pow(10, places);
        return Math.round(value * scale) / scale;
    }
}