package frc.robot.library.vision.limelight;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.library.bling.SubSys_Bling;
import frc.robot.library.drivetrains.swerve_ctre.SubSys_SwerveDrive;
import lombok.Setter;

import java.util.Map;

import static frc.robot.library.vision.limelight.SubSys_Limelight_Constants.*;

/**
 * Subsystem for interfacing with the Limelight vision system.
 */
public class SubSys_Limelight extends SubsystemBase {
    @Setter
    private SubSys_Bling subSysBling;
    @Setter
    private SubSys_SwerveDrive subSysDrive;
    private final DetectedObjectList detectedObjectList = new DetectedObjectList();
    private final ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");
    private final ShuffleboardLayout detectedObjectsLayout;

    /**
     * Constructs a new SubSys_Limelight.
     *
     * @param subSysBling The Bling subsystem for visual feedback.
     * @param subSysDrive The SwerveDrive subsystem for pose estimation.
     */
    public SubSys_Limelight(SubSys_Bling subSysBling, SubSys_SwerveDrive subSysDrive) {
        this.subSysBling = subSysBling;
        this.subSysDrive = subSysDrive;
        DetectedObject.setDrive(subSysDrive);

        detectedObjectsLayout = limelightTab
                .getLayout("Detected Objects", BuiltInLayouts.kGrid)
                .withProperties(Map.of("Number of columns", 1, "Number of rows", 3));

        initializeShuffleboard();
    }

    /**
     * Initializes Shuffleboard displays for the Limelight subsystem.
     */
    private void initializeShuffleboard() {
        detectedObjectsLayout.addStringArray("Highest Confidence", () -> detectedObjectList.sortByConfidence().toArray());

        if (subSysDrive != null) {
            detectedObjectsLayout.addStringArray("Closest To Robot", () -> detectedObjectList.sortByPose(subSysDrive.getState().Pose).toArray());
        } else {
            detectedObjectsLayout.addString("Closest To Robot", () -> "Cannot get Pose from drive!");
        }

        limelightTab.addBoolean("Limelight Connected", this::isLimelightConnected);
        limelightTab.addBoolean("Object Detected", () -> LimelightLib.getTV(NN_LIMELIGHT));
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
        if (OBJECT_DETECTION_ENABLED && isLimelightConnected()) {
            updateDetectedObjects();
        }

        detectedObjectList.update();
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
                detectedObjectList.add(new DetectedObjectList.DetectedObjectPair(note, detection.confidence));
            }
        }
    }
}