package frc.alotobots.library.vision.limelight;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.bling.BlingSubsystem;
import frc.alotobots.library.drivetrains.swerve_ctre.SwerveDriveSubsystem;
import lombok.Setter;

import java.util.Map;

import static frc.alotobots.library.vision.limelight.LimelightSubsystemConstants.*;

/**
 * Subsystem for interfacing with the Limelight vision system.
 */
public class LimelightSubsystem extends SubsystemBase {
    @Setter
    private BlingSubsystem subSysBling;
    @Setter
    private SwerveDriveSubsystem subSysDrive;
    private DetectedObjectList detectedObjectList;
    private ShuffleboardTab limelightTab;
    private ShuffleboardLayout detectedObjectsLayout;

    /**
     * Constructs a new SubSys_Limelight.
     *
     * @param subSysBling The Bling subsystem for visual feedback.
     * @param subSysDrive The SwerveDrive subsystem for pose estimation.
     */
    public LimelightSubsystem(BlingSubsystem subSysBling, SwerveDriveSubsystem subSysDrive) {
        System.out.println("Initializing LimelightSubsystem");
        this.subSysBling = subSysBling;
        this.subSysDrive = subSysDrive;
        DetectedObject.setDrive(subSysDrive);
        System.out.println("LimelightSubsystem initialized");
    }

    private void initializeShuffleboard() {
        if (limelightTab == null) {
            System.out.println("Initializing Limelight Shuffleboard");
            limelightTab = Shuffleboard.getTab("Limelight");
            detectedObjectsLayout = limelightTab
                    .getLayout("Detected Objects", BuiltInLayouts.kGrid)
                    .withProperties(Map.of("Number of columns", 1, "Number of rows", 3));

            detectedObjectsLayout.addStringArray("Highest Confidence", () -> getDetectedObjectList().sortByConfidence().toArray());

            if (subSysDrive != null) {
                detectedObjectsLayout.addStringArray("Closest To Robot", () -> getDetectedObjectList().sortByPose(subSysDrive.getState().Pose).toArray());
            } else {
                detectedObjectsLayout.addString("Closest To Robot", () -> "Cannot get Pose from drive!");
            }

            limelightTab.addBoolean("Limelight Connected", this::isLimelightConnected);
            limelightTab.addBoolean("Object Detected", () -> LimelightLib.getTV(NN_LIMELIGHT));
            System.out.println("Limelight Shuffleboard initialized");
        }
    }

    private DetectedObjectList getDetectedObjectList() {
        if (detectedObjectList == null) {
            System.out.println("Initializing DetectedObjectList");
            detectedObjectList = new DetectedObjectList();
            System.out.println("DetectedObjectList initialized");
        }
        return detectedObjectList;
    }

    /**
     * Calculates the distance to a target using the camera's parameters.
     *
     * @param targetHeight The height of the target from the floor in meters.
     * @param targetOffsetAngle_Vertical The vertical offset angle to the target in radians.
     * @return The distance to the target in meters.
     */
    public double targetDistanceMetersCamera(double targetHeight, double targetOffsetAngle_Vertical) {
        double angleToGoalRadians = LL_OFFSET.getRotation().getY() + targetOffsetAngle_Vertical;
        return (targetHeight - LL_OFFSET.getZ()) / Math.tan(angleToGoalRadians);
    }

    /**
     * Checks if the Limelight is connected and responding.
     *
     * @return true if the Limelight is connected, false otherwise.
     */
    private boolean isLimelightConnected() {
        return !NetworkTableInstance.getDefault()
                .getTable(NN_LIMELIGHT)
                .getEntry("json")
                .getString("")
                .isEmpty();
    }

    @Override
    public void periodic() {
        initializeShuffleboard();
        if (OBJECT_DETECTION_ENABLED && isLimelightConnected()) {
            updateDetectedObjects();
        }

        getDetectedObjectList().update();
    }

    /**
     * Updates the list of detected objects based on the latest Limelight data.
     */
    private void updateDetectedObjects() {
        boolean objectDetected = LimelightLib.getTV(NN_LIMELIGHT);
        if (objectDetected) {
            LimelightLib.LimelightResults latestResults = LimelightLib.getLatestResults(NN_LIMELIGHT);

            for (LimelightLib.LimelightTarget_Detector detection : latestResults.targetingResults.targets_Detector) {
                double horizontalOffset = Math.toRadians(detection.tx);
                double verticalOffset = Math.toRadians(-detection.ty); // Make CCW positive

                DetectedObject note = new DetectedObject(horizontalOffset, verticalOffset, DetectedObject.ObjectType.NOTE, LL_OFFSET);
                getDetectedObjectList().add(new DetectedObjectList.DetectedObjectPair(note, detection.confidence));
            }
        }
    }
}