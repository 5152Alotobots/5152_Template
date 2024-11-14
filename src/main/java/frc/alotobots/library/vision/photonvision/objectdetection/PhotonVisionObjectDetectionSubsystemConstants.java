package frc.alotobots.library.vision.photonvision.objectdetection;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import lombok.experimental.UtilityClass;
import org.photonvision.PhotonCamera;

@SuppressWarnings("resource")
@UtilityClass
public class PhotonVisionObjectDetectionSubsystemConstants {
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
  public static final GameElement[] GAME_ELEMENTS =
      new GameElement[] {new GameElement("Note", .36, .36, .05, 0)};

  // Telemetry Update Rate (matches AprilTag subsystem)
  public static final double TELEMETRY_UPDATE_RATE = 0.05; // 50ms update rate
  public static final long TELEMETRY_UPDATE_TIME = (long)(TELEMETRY_UPDATE_RATE * 1000); // in milliseconds
}
