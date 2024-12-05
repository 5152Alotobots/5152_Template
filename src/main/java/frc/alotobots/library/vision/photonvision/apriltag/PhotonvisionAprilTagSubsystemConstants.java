package frc.alotobots.library.vision.photonvision.apriltag;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import lombok.experimental.UtilityClass;
import org.photonvision.PhotonCamera;

@SuppressWarnings("resource")
@UtilityClass
public class PhotonvisionAprilTagSubsystemConstants {

  public static final double FIELD_LENGTH = 16.54175; // in meters

  // OFFSETS
  // FORWARD: +, LEFT: +, UP: + (USED FOR APRILTAGS)
  public static final Transform3d[] CAMERA_OFFSETS =
      new Transform3d[] {
        // Front Left
        new Transform3d(
            new Translation3d(0.245, 0.21, 0.17),
            new Rotation3d(0, Math.toRadians(-35), Math.toRadians(45))),
        // Front Middle
        new Transform3d(
            new Translation3d(0.275, 0.0, 0.189),
            new Rotation3d(0, Math.toRadians(-35), Math.toRadians(0)))
      };
  // CAMERAS
  public static final PhotonCamera[] CAMERAS =
      new PhotonCamera[] {
        new PhotonCamera("FL_AprilTag"), // Front Left
        new PhotonCamera("FM_AprilTag"), // Front Middle
        // new PhotonCamera("BL_AprilTag"),
        // new PhotonCamera("BR_AprilTag")
      };

  // IOs
  public static final AprilTagIOPhotonVision[] = new AprilTagIOPhotonVision[]{

  }

  public static final boolean USE_VISION_POSE_ESTIMATION = true;
  public static final boolean ONLY_USE_POSE_ESTIMATION_IN_TELEOP = false;

  /*
   * The standard deviations to use for the vision
   */
  // NEEDS TO BE COPIED FROM PHOTONVISION ON COMPETITION DAY!!
  public static final Matrix<N3, N1> VISION_STD_DEVS =
      MatBuilder.fill(
          Nat.N3(),
          Nat.N1(),
          0.001, // x in meters (default=0.9)
          0.001, // y in meters (default=0.9)
          0.1 // heading in radians. The gyroscope is very accurate, so as long as it is reset
          // correctly it is unnecessary to correct it with vision (set this to a large value like
          // 1000)
          );

  // Pose estimation constants
  public static final double MAX_POSE_DEVIATION_METERS =
      1.0; // Maximum allowed deviation from median
  public static final double MIN_TAG_WEIGHT = 0.3; // Minimum weight for single tag poses
  public static final double MAX_TAG_WEIGHT = 1.0; // Maximum weight for multi-tag poses

  // Smoothing constants
  public static final double POSITION_ALPHA = 0.05; // Lower = more smoothing (0-1)
  public static final double ROTATION_ALPHA = 0.05; // Lower = more smoothing (0-1)
}
