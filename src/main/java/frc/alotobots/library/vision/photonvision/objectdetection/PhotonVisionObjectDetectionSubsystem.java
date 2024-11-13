package frc.alotobots.library.vision.photonvision.objectdetection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.List;
import static frc.alotobots.library.vision.photonvision.objectdetection.PhotonVisionObjectDetectionSubsystemConstants.*;

public class PhotonVisionObjectDetectionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private List<PhotonTrackedTarget> latestTargets;
    private final PhotonVisionObjectDetectionTelemetry telemetry;

    public PhotonVisionObjectDetectionSubsystem() {
        camera = new PhotonCamera(CAMERA_NAME);
        telemetry = new PhotonVisionObjectDetectionTelemetry();
    }

    @Override
    public void periodic() {
        var result = camera.getLatestResult();
        
        if (result.hasTargets()) {
            latestTargets = result.getTargets();
            // Filter targets based on confidence and distance
            latestTargets.removeIf(target -> 
                target.getPoseAmbiguity() > MAX_POSE_AMBIGUITY ||
                target.getBestCameraToTarget().getTranslation().getNorm() > MAX_TRACK_DISTANCE
            );
        } else {
            latestTargets = List.of();
        }

        telemetry.update(this);
    }

    public List<PhotonTrackedTarget> getLatestTargets() {
        return latestTargets;
    }

    public boolean hasTargets() {
        return latestTargets != null && !latestTargets.isEmpty();
    }

    public PhotonCamera getCamera() {
        return camera;
    }
}
