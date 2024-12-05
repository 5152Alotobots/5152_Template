package frc.alotobots.library.vision.photonvision.apriltag;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

public class AprilTagIOPhotonVision implements AprilTagIO {
    protected final PhotonCamera camera;
    protected final Transform3d robotToCamera;

    public AprilTagIOPhotonVision(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
    }

    @Override
    public void updateInputs(AprilTagIOInputs inputs) {
        // Log if the camera is connected or not
        inputs.connected = camera.isConnected();
        // Log PhotonPipeline results
        var results = camera.getAllUnreadResults();
        inputs.observations = new PhotonPipelineResult[results.size()];
        int i = 0;
        for (var result : results) {
            inputs.observations[i++] = result;
        }
    }
}
