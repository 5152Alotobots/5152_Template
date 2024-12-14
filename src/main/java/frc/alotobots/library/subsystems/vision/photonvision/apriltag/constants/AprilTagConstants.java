//   ____  _ ____ ____       _    _        ___ _____ ___  ____     ___ _____ ____
//  | ___|/ | ___|___ \    / \  | |     / _ \_   _/ _ \| __) / _ \_   _/ ___|
//  |___ \| |___ \ __) | / _ \ | |    | | | || || | | |  _ \|| | || | \___ \
//   ___) | |___) / __/   / ___ \| |__  | |_| || || |_| | |_) | |_| || |  ___) |
//  |____/|_|____/_____| /_/   \_\_____\___/ |_| \___/|____/ \___/ |_| |____/
//
//
// 2025 ALOTOBOTS FRC 5152
// Robot Code
package frc.alotobots.library.subsystems.vision.photonvision.apriltag.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import lombok.experimental.UtilityClass;
import org.photonvision.simulation.SimCameraProperties;

@SuppressWarnings("resource")
@UtilityClass
public class AprilTagConstants {
  // AprilTag layout
  public static AprilTagFieldLayout APRIL_TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // OFFSETS
  // FORWARD: +, LEFT: +, UP: + (USED FOR APRILTAGS)
  private static final Transform3d[] CAMERA_OFFSETS =
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
  public static final CameraConfig[] CAMERA_CONFIGS = {
    new CameraConfig("FL_AprilTag", CAMERA_OFFSETS[0], new SimCameraProperties()),
    new CameraConfig("FM_AprilTag", CAMERA_OFFSETS[1], new SimCameraProperties())
  };

  /*
   * The standard deviations to use for the vision
   */
  // NEEDS TO BE COPIED FROM PHOTONVISION ON COMPETITION DAY!!
  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  // LOWER = TRUST MORE
  public static double LINEAR_STD_DEV_BASE = 0.02; // Meters
  public static double ANGULAR_STD_DEV_BASE = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  // SHOULD NEVER BE LESS THAN 1.0, NUMBERS GREATER THAN 1 = TRUST LESS
  public static double[] CAMERA_STD_DEV_FACTORS = new double[] {1.0, 1.0};

  // Basic Filtering
  public static double MAX_AMBIGUITY = 0.3;
  public static double MAX_Z_ERROR = 0.75;
}
