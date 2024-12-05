package frc.alotobots.library.vision.photonvision.apriltag;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

public interface AprilTagIO {
  @AutoLog
  class AprilTagIOInputs {
    public boolean connected = false;
    public PhotonPipelineResult[] observations =
        new PhotonPipelineResult[0];
  }

  default void updateInputs(AprilTagIOInputs inputs) {}
}
