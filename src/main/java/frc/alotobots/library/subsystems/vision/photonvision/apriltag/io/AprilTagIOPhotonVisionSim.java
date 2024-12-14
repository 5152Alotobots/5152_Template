//   ____  _ ____ ____       _    _        ___ _____ ___  ____     ___ _____ ____
//  | ___|/ | ___|___ \    / \  | |     / _ \_   _/ _ \| __) / _ \_   _/ ___|
//  |___ \| |___ \ __) | / _ \ | |    | | | || || | | |  _ \|| | || | \___ \
//   ___) | |___) / __/   / ___ \| |__  | |_| || || |_| | |_) | |_| || |  ___) |
//  |____/|_|____/_____| /_/   \_\_____\___/ |_| \___/|____/ \___/ |_| |____/
//
//
// 2025 ALOTOBOTS FRC 5152
// Robot Code
package frc.alotobots.library.subsystems.vision.photonvision.apriltag.io;

import static frc.alotobots.library.subsystems.vision.photonvision.apriltag.constants.AprilTagConstants.APRIL_TAG_LAYOUT;

import edu.wpi.first.math.geometry.Pose2d;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.constants.CameraConfig;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

public class AprilTagIOPhotonVisionSim extends AprilTagIOPhotonVision {
  private static VisionSystemSim visionSim;
  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  public AprilTagIOPhotonVisionSim(CameraConfig config, Supplier<Pose2d> poseSupplier) {
    super(config);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(APRIL_TAG_LAYOUT);
    }

    // Add sim camera
    cameraSim = new PhotonCameraSim(camera, config.properties());
    visionSim.addCamera(cameraSim, config.robotToCamera());
  }

  @Override
  public void updateInputs(AprilTagIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
