package frc.alotobots.library.vision.photonvision.objectdetection;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import lombok.experimental.UtilityClass;
import org.photonvision.PhotonCamera;

@SuppressWarnings("resource")
@UtilityClass
public class PhotonVisionObjectDetectionSubsystemConstants {
  public static final boolean USE_OBJECT_DETECTION = true;
  public static final boolean ONLY_USE_OBJECT_DETECTION_IN_TELEOP = false;
  // Camera Configuration
  public static final PhotonCamera[] CAMERAS =
      new PhotonCamera[] {
        new PhotonCamera("FM_ObjectDetection") // Front Middle
      };

  // Camera mounting offsets (FORWARD: +, LEFT: +, UP: +)
  public static final Transform3d[] CAMERA_OFFSETS =
      new Transform3d[] {
        // Front Middle
        new Transform3d(
            new Translation3d(0.275, 0.0, 0.23),
            new Rotation3d(0, Math.toRadians(0), Math.toRadians(0)))
      };
  // Game elements array indexed by class ID
  // Index 0 = Note (class ID 0)
  public static final GameElement[] GAME_ELEMENTS =
      new GameElement[] {
        new GameElement("Note", .36, .36, .05) // Class ID 0 (represented by array index)
      };

  // Object tracking constants
  public static final double INITIAL_CONFIDENCE = 0.99;
  public static final double CONFIDENCE_DECAY_RATE = 0.33;
  public static final double MIN_CONFIDENCE = 0.0;
  public static final double POSITION_MATCH_TOLERANCE = 0.5; // meters
}
