//   ____  _ ____ ____       _    _        ___ _____ ___  ____     ___ _____ ____
//  | ___|/ | ___|___ \    / \  | |     / _ \_   _/ _ \| __) / _ \_   _/ ___|
//  |___ \| |___ \ __) | / _ \ | |    | | | || || | | |  _ \|| | || | \___ \
//   ___) | |___) / __/   / ___ \| |__  | |_| || || |_| | |_) | |_| || |  ___) |
//  |____/|_|____/_____| /_/   \_\_____\___/ |_| \___/|____/ \___/ |_| |____/
//
//
// 2025 ALOTOBOTS FRC 5152
// Robot Code
package frc.alotobots.library.subsystems.vision.photonvision.objectdetection.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.util.GameElement;
import lombok.experimental.UtilityClass;
import org.photonvision.simulation.SimCameraProperties;

@SuppressWarnings("resource")
@UtilityClass
public class ObjectDetectionConstants {
  // Camera mounting offsets (FORWARD: +, LEFT: +, UP: +)
  private static final Transform3d[] CAMERA_OFFSETS =
      new Transform3d[] {
        // Front Middle
        new Transform3d(
            new Translation3d(0.275, 0.0, 0.23),
            new Rotation3d(0, Math.toRadians(0), Math.toRadians(0)))
      };

  // Camera configurations
  public static final CameraConfig[] CAMERA_CONFIGS = {
    new CameraConfig("FM_ObjectDetection", CAMERA_OFFSETS[0], new SimCameraProperties())
  };

  // Game elements array indexed by class ID
  // Index 0 = Note (class ID 0)
  public static final GameElement[] GAME_ELEMENTS =
      new GameElement[] {
        new GameElement("Note", .36, .36, .05) // Class ID 0 (represented by array index)
      };

  // TUNE ON COMP. DAY!!
  /**
   * The number of detections to save in history, the smaller the better, but still needs to be
   * enough to ensure we have a good detection
   */
  public static final int HISTORY_LENGTH = 30;

  /**
   * Number of detections in history that need to be detected with that object for it to be
   * considered stable
   */
  public static final int REQUIRED_DETECTIONS = 10;

  /**
   * Number of missed detections IN A ROW of a stable object for it to be removed from the stable
   * list
   */
  public static final int MISSING_FRAMES_THRESHOLD = 10;

  /*** Distance between objects considered as the same */
  public static final double POSITION_MATCH_TOLERANCE = 0.2; // meters
}
