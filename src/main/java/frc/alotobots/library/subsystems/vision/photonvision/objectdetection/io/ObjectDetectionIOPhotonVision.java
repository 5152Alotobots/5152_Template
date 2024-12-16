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

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.constants.CameraConfig;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.constants.ObjectDetectionConstants;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.util.GameElement;
import java.util.LinkedList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class ObjectDetectionIOPhotonVision implements ObjectDetectionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  public ObjectDetectionIOPhotonVision(CameraConfig config) {
    this.camera = new PhotonCamera(config.name());
    this.robotToCamera = config.robotToCamera();

    if (ObjectDetectionConstants.GAME_ELEMENTS.length == 0) {
      throw new IllegalStateException("No game elements configured while running ObjectDetection!");
    }
  }

  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    List<DetectedObject> detectedObjects = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // First check if we have targets
      if (result.hasTargets()) {
        // Loop through each result
        for (var target : result.getTargets()) {
          // Basic data
          int classId = target.getDetectedObjectClassID();
          float confidence = target.getDetectedObjectConfidence();

          // Make sure we have the object in our list of game elements
          if (ObjectDetectionConstants.GAME_ELEMENTS[target.getDetectedObjectClassID()] == null) {
            throw new IllegalStateException("No object detected for class: " + classId);
          }

          // Match object with list of game elements
          GameElement matchedElement = ObjectDetectionConstants.GAME_ELEMENTS[classId];

          // Compute robot relative pose
          // Calculate distance using the matched element's height
          double targetToCameraDistance =
              PhotonUtils.calculateDistanceToTargetMeters(
                  robotToCamera.getZ(),
                  matchedElement.height(),
                  robotToCamera.getRotation().getY(),
                  Units.degreesToRadians(target.getPitch()));

          // Get the 2D translation in camera space
          Translation2d targetToCamera2d =
              PhotonUtils.estimateCameraToTargetTranslation(
                  targetToCameraDistance, Rotation2d.fromDegrees(-target.getYaw()));

          // Convert the 2D translation to a 3D transform, assume that z is 0 as object is on the
          // ground
          Transform3d cameraToTarget =
              new Transform3d(
                  new Translation3d(targetToCamera2d.getX(), targetToCamera2d.getY(), 0),
                  new Rotation3d());

          // Now combine the transforms to get target in robot space
          Transform3d targetToRobot = robotToCamera.plus(cameraToTarget);

          // Finally add to the array
          detectedObjects.add(
              new DetectedObject(result.getTimestampSeconds(), targetToRobot, confidence, classId));
        }
      }
    }

    // Save detected objects to inputs object
    inputs.detectedObjects = new ObjectDetectionIO.DetectedObject[detectedObjects.size()];
    for (int i = 0; i < detectedObjects.size(); i++) {
      inputs.detectedObjects[i] = detectedObjects.get(i);
    }
  }
}
