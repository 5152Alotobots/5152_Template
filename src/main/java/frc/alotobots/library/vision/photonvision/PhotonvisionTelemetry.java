package frc.alotobots.library.vision.photonvision;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSource;
import frc.alotobots.Constants;

import java.util.Map;
import java.util.Optional;

/**
 * Handles telemetry for the Photonvision subsystem.
 */
public class PhotonvisionTelemetry {
    private final ShuffleboardTab photonvisionTab;
    private final GenericEntry poseEstimateEntry;

    /**
     * Constructs a new PhotonvisionTelemetry object.
     */
    public PhotonvisionTelemetry() {
        this.photonvisionTab = Shuffleboard.getTab("Photonvision");
        this.poseEstimateEntry = initializeShuffleboard();
    }

    /**
     * Initializes the Shuffleboard layout for Photonvision telemetry.
     *
     * @return The GenericEntry for pose estimates.
     */
    private GenericEntry initializeShuffleboard() {
        GenericEntry poseEstimateEntry = photonvisionTab.add("Vision Pose Estimate", "Not available")
                .withPosition(0, 0)
                .withSize(2, 1)
                .getEntry();

        photonvisionTab.addBoolean("Vision Pose Estimation Enabled", () -> PhotonvisionSubsystemConstants.USE_VISION_POSE_ESTIMATION)
                .withPosition(0, 1);
        photonvisionTab.addBoolean("Only Use Pose Estimation in Teleop", () -> PhotonvisionSubsystemConstants.ONLY_USE_POSE_ESTIMATION_IN_TELEOP)
                .withPosition(0, 2);

        // Add Photonvision camera stream
        HttpCamera photonCamera = new HttpCamera("Photonvision", "http://photonvision.local:1182/stream.mjpg");
        photonCamera.setFPS(20);
        photonvisionTab.add("Photonvision Camera", photonCamera)
                .withWidget(BuiltInWidgets.kCameraStream)
                .withPosition(2, 0)
                .withSize(4, 3);

        return poseEstimateEntry;
    }

    /**
     * Updates the Shuffleboard with the latest telemetry data.
     *
     * @param estimatedPose The estimated pose from Photonvision.
     */
    public void updateShuffleboard(Optional<Pose2d> estimatedPose) {
        poseEstimateEntry.setString(estimatedPose.map(Pose2d::toString).orElse("NONE"));
    }
}