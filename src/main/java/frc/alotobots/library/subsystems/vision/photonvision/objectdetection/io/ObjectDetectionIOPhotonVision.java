//   ____  _ ____ ____       _    _        ___ _____ ___  ____     ___ _____ ____
//  | ___|/ | ___|___ \    / \  | |     / _ \_   _/ _ \| __) / _ \_   _/ ___|
//  |___ \| |___ \ __) | / _ \ | |    | | | || || | | |  _ \|| | || | \___ \
//   ___) | |___) / __/   / ___ \| |__  | |_| || || |_| | |_) | |_| || |  ___) |
//  |____/|_|____/_____| /_/   \_\_____\___/ |_| \___/|____/ \___/ |_| |____/
//
//
// 2025 ALOTOBOTS FRC 5152
// Robot Code
package frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.constants.CameraConfig;
import org.photonvision.PhotonCamera;

/** Real implementation of ObjectDetectionIO using PhotonVision. */
public class ObjectDetectionIOPhotonVision implements ObjectDetectionIO {
  private final PhotonCamera camera;
  private final Transform3d cameraOffset;

  public ObjectDetectionIOPhotonVision(CameraConfig config) {
    this.camera = new PhotonCamera(config.name());
    this.cameraOffset = config.robotToCamera();
  }

  @Override
  public void updateInputs(ObjectDetectionInputs inputs) {
    inputs.timestamp = Timer.getFPGATimestamp();

    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      var result = results.get(0);
      inputs.hasTargets = result.hasTargets();

      var targets = result.getTargets();
      int numTargets = targets.size();

      // Initialize arrays with the correct size based on number of targets
      inputs.targetYaws = new double[numTargets];
      inputs.targetPitches = new double[numTargets];
      inputs.targetClassIds = new int[numTargets];
      inputs.targetAreas = new double[numTargets];

      // Fill arrays with target data
      for (int i = 0; i < numTargets; i++) {
        var target = targets.get(i);
        inputs.targetYaws[i] = target.getYaw();
        inputs.targetPitches[i] = target.getPitch();
        inputs.targetClassIds[i] = target.getDetectedObjectClassID();
        inputs.targetAreas[i] = target.getArea();
      }

      inputs.cameraToRobot = cameraOffset;
    }
  }
}
