package frc.alotobots.library.vision.photonvision;

import static frc.alotobots.library.vision.photonvision.PhotonvisionSubsystemConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * A subsystem that manages multiple PhotonVision cameras and provides pose estimation
 * functionality.
 */
public class PhotonvisionSubsystem extends SubsystemBase {
  private final AprilTagFieldLayout aprilTagFieldLayout;
  private ArrayList<PhotonPoseEstimator> photonPoseEstimators;
  private final PhotonvisionTelemetry telemetry;
  private final boolean[] cameraEnabled;

  // Smoothing filter state
  private Pose3d lastSmoothedPose;
  private static final double POSITION_ALPHA = 0.3; // Lower = more smoothing
  private static final double ROTATION_ALPHA = 0.2; // Lower = more smoothing

  /** Constructs a new PhotonvisionSubsystem. */
  public PhotonvisionSubsystem() {
    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    photonPoseEstimators = new ArrayList<>();
    cameraEnabled = new boolean[CAMERAS.length];
    Arrays.fill(cameraEnabled, true);
    initializePoseEstimators();
    telemetry = new PhotonvisionTelemetry();
  }

  /**
   * Initializes pose estimators for each camera in the system. Creates PhotonPoseEstimator objects
   * configured for multi-tag detection and sets up fallback strategies. Throws RuntimeException if
   * camera offsets are not properly configured.
   */
  private void initializePoseEstimators() {
    photonPoseEstimators = new ArrayList<>();

    if (CAMERAS.length != CAMERA_OFFSETS.length) {
      throw new RuntimeException(
          "PhotonCamera object is missing offset! Did you add an offset in Photonvision_Constants?");
    }

    // Initialize one estimator per camera, even if not connected
    for (int i = 0; i < CAMERAS.length; i++) {
      if (CAMERAS[i] != null) {
        PhotonPoseEstimator estimator =
            new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                CAMERAS[i],
                CAMERA_OFFSETS[i]);
        estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        photonPoseEstimators.add(estimator);

        cameraEnabled[i] = CAMERAS[i].isConnected();
      } else {
        photonPoseEstimators.add(null);
        cameraEnabled[i] = false;
      }
    }
  }

  @Override
  public void periodic() {
    if (USE_VISION_POSE_ESTIMATION) {
      Optional<Pair<Pose2d, Double>> estimatedPose = getEstimatedVisionPose2d();
      List<PhotonTrackedTarget> detectedTags = getDetectedTags();
      List<Pair<Integer, Pair<Pose3d, Double>>> perCameraPoses = getPerCameraEstimatedPoses();
      telemetry.updateShuffleboard(estimatedPose.map(Pair::getFirst), detectedTags, perCameraPoses);
    }
  }

  /**
   * Returns a list of all detected AprilTags from all cameras.
   *
   * @return A list of PhotonTrackedTarget objects representing the detected AprilTags.
   */
  /**
   * Returns a list of all detected AprilTags from all enabled and connected cameras. Combines
   * results from multiple cameras into a single list.
   *
   * @return A list of PhotonTrackedTarget objects representing all detected AprilTags
   */
  private List<PhotonTrackedTarget> getDetectedTags() {
    List<PhotonTrackedTarget> allDetectedTags = new ArrayList<>();
    for (PhotonCamera camera : CAMERAS) {
      var result = camera.getLatestResult();
      if (result.hasTargets()) {
        allDetectedTags.addAll(result.getTargets());
      }
    }
    return allDetectedTags;
  }

  /**
   * Returns the estimated 3D robot pose using the MultiTag pose from the coprocessor, if available.
   *
   * @param previousPose The previous pose of the robot.
   * @return A Pair containing the estimated Pose3d and the timestamp of the estimation, or an empty
   *     Optional if the pose is not available.
   */
  public Optional<Pair<Pose3d, Double>> getEstimatedVisionPose3d(Pose2d previousPose) {
    ArrayList<EstimatedRobotPose> estimates = new ArrayList<>();
    for (int i = 0; i < photonPoseEstimators.size(); i++) {
      PhotonPoseEstimator estimator = photonPoseEstimators.get(i);
      if (estimator != null && cameraEnabled[i] && CAMERAS[i].isConnected()) {
        estimator.setLastPose(previousPose);
        var estimate = estimator.update();
        if (estimate.isPresent()) {
          estimates.add(estimate.get());
        }
      }
    }
    return averageEstimates(estimates);
  }

  /**
   * Returns the estimated 3D robot pose using the MultiTag pose from the coprocessor, if available.
   *
   * @return A Pair containing the estimated Pose3d and the timestamp of the estimation, or an empty
   *     Optional if the pose is not available.
   */
  public Optional<Pair<Pose3d, Double>> getEstimatedVisionPose3d() {
    ArrayList<EstimatedRobotPose> estimates = new ArrayList<>();
    for (int i = 0; i < photonPoseEstimators.size(); i++) {
      PhotonPoseEstimator estimator = photonPoseEstimators.get(i);
      if (estimator != null && cameraEnabled[i] && CAMERAS[i].isConnected()) {
        var estimate = estimator.update();
        if (estimate.isPresent()) {
          estimates.add(estimate.get());
        }
      }
    }
    return averageEstimates(estimates);
  }

  /**
   * Returns the estimated 2D robot pose using the MultiTag pose from the coprocessor, if available.
   *
   * @param previousPose The previous pose of the robot.
   * @return A Pair containing the estimated Pose2d and the timestamp of the estimation, or an empty
   *     Optional if the pose is not available.
   */
  public Optional<Pair<Pose2d, Double>> getEstimatedVisionPose2d(Pose2d previousPose) {
    return getEstimatedVisionPose3d(previousPose)
        .map(pair -> new Pair<>(pair.getFirst().toPose2d(), pair.getSecond()));
  }

  /**
   * Returns the estimated 2D robot pose using the MultiTag pose from the coprocessor, if available.
   *
   * @return A Pair containing the estimated Pose2d and the timestamp of the estimation, or an empty
   *     Optional if the pose is not available.
   */
  public Optional<Pair<Pose2d, Double>> getEstimatedVisionPose2d() {
    return getEstimatedVisionPose3d()
        .map(pair -> new Pair<>(pair.getFirst().toPose2d(), pair.getSecond()));
  }

  private static final double MAX_POSE_DEVIATION_METERS =
      1.0; // Maximum allowed deviation from median
  private static final double MIN_TAG_WEIGHT = 0.3; // Minimum weight for single tag poses
  private static final double MAX_TAG_WEIGHT = 1.0; // Maximum weight for multi-tag poses

  /**
   * Processes and averages the estimated robot poses from multiple cameras with outlier rejection
   * and tag-count weighting.
   *
   * @param estimates An ArrayList of EstimatedRobotPose objects.
   * @return A Pair containing the weighted average Pose3d and timestamp, or an empty Optional if no
   *     valid poses.
   */
  private Optional<Pair<Pose3d, Double>> averageEstimates(ArrayList<EstimatedRobotPose> estimates) {
    if (estimates.isEmpty()) {
      return Optional.empty();
    }

    // First, filter out obvious outliers based on position
    List<EstimatedRobotPose> validEstimates = removeOutliers(estimates);

    if (validEstimates.isEmpty()) {
      return Optional.empty();
    }

    // Calculate weights based on number of tags and ambiguity
    double[] weights = calculateWeights(validEstimates);

    // Compute weighted average pose
    double x = 0, y = 0, z = 0;
    double rotX = 0, rotY = 0, rotZ = 0;
    double timestamp = 0;
    double totalWeight = 0;

    for (int i = 0; i < validEstimates.size(); i++) {
      Pose3d pose = validEstimates.get(i).estimatedPose;
      double weight = weights[i];

      x += pose.getX() * weight;
      y += pose.getY() * weight;
      z += pose.getZ() * weight;

      // Handle rotation averaging properly
      rotX += pose.getRotation().getX() * weight;
      rotY += pose.getRotation().getY() * weight;
      rotZ += pose.getRotation().getZ() * weight;

      timestamp += validEstimates.get(i).timestampSeconds * weight;
      totalWeight += weight;
    }

    // Normalize by total weight
    x /= totalWeight;
    y /= totalWeight;
    z /= totalWeight;
    rotX /= totalWeight;
    rotY /= totalWeight;
    rotZ /= totalWeight;
    timestamp /= totalWeight;

    Pose3d averagePose = new Pose3d(x, y, z, new Rotation3d(rotX, rotY, rotZ));

    // Apply smoothing filter
    if (lastSmoothedPose == null) {
      lastSmoothedPose = averagePose;
    } else {
      // Smooth position
      double smoothedX =
          exponentialSmooth(lastSmoothedPose.getX(), averagePose.getX(), POSITION_ALPHA);
      double smoothedY =
          exponentialSmooth(lastSmoothedPose.getY(), averagePose.getY(), POSITION_ALPHA);
      double smoothedZ =
          exponentialSmooth(lastSmoothedPose.getZ(), averagePose.getZ(), POSITION_ALPHA);

      // Smooth rotation
      double smoothedRotX =
          exponentialSmooth(
              lastSmoothedPose.getRotation().getX(),
              averagePose.getRotation().getX(),
              ROTATION_ALPHA);
      double smoothedRotY =
          exponentialSmooth(
              lastSmoothedPose.getRotation().getY(),
              averagePose.getRotation().getY(),
              ROTATION_ALPHA);
      double smoothedRotZ =
          exponentialSmooth(
              lastSmoothedPose.getRotation().getZ(),
              averagePose.getRotation().getZ(),
              ROTATION_ALPHA);

      lastSmoothedPose =
          new Pose3d(
              smoothedX,
              smoothedY,
              smoothedZ,
              new Rotation3d(smoothedRotX, smoothedRotY, smoothedRotZ));
    }

    return Optional.of(new Pair<>(lastSmoothedPose, timestamp));
  }

  /**
   * Removes outlier poses based on their distance from the median position.
   *
   * @param estimates List of pose estimates to filter
   * @return Filtered list with outliers removed
   */
  private List<EstimatedRobotPose> removeOutliers(List<EstimatedRobotPose> estimates) {
    if (estimates.size() <= 2) {
      return estimates;
    }

    // Calculate median X and Y positions
    double[] xPositions =
        estimates.stream().mapToDouble(e -> e.estimatedPose.getX()).sorted().toArray();
    double[] yPositions =
        estimates.stream().mapToDouble(e -> e.estimatedPose.getY()).sorted().toArray();

    double medianX = xPositions[xPositions.length / 2];
    double medianY = yPositions[yPositions.length / 2];

    // Filter out poses that are too far from the median
    return estimates.stream()
        .filter(
            estimate -> {
              double dx = estimate.estimatedPose.getX() - medianX;
              double dy = estimate.estimatedPose.getY() - medianY;
              double distance = Math.sqrt(dx * dx + dy * dy);
              return distance <= MAX_POSE_DEVIATION_METERS;
            })
        .collect(Collectors.toList());
  }

  /**
   * Calculates weights for each pose estimate based on number of tags and ambiguity.
   *
   * @param estimates List of pose estimates
   * @return Array of weights corresponding to each estimate
   */
  private double[] calculateWeights(List<EstimatedRobotPose> estimates) {
    double[] weights = new double[estimates.size()];

    for (int i = 0; i < estimates.size(); i++) {
      EstimatedRobotPose estimate = estimates.get(i);
      int numTags = estimate.targetsUsed.size();

      // Base weight on number of tags seen
      double weight =
          MIN_TAG_WEIGHT + (MAX_TAG_WEIGHT - MIN_TAG_WEIGHT) * Math.min(1.0, (numTags - 1) / 3.0);

      // Reduce weight if ambiguity is high
      double avgAmbiguity =
          estimate.targetsUsed.stream()
              .mapToDouble(target -> target.getPoseAmbiguity())
              .average()
              .orElse(0.0);
      weight *= (1.0 - Math.min(1.0, avgAmbiguity));

      weights[i] = weight;
    }

    return weights;
  }

  /**
   * Applies exponential smoothing to a value.
   *
   * @param oldValue Previous smoothed value
   * @param newValue New raw value
   * @param alpha Smoothing factor (0-1), lower = more smoothing
   * @return Smoothed value
   */
  private double exponentialSmooth(double oldValue, double newValue, double alpha) {
    return alpha * newValue + (1.0 - alpha) * oldValue;
  }

  /**
   * Returns the estimated 3D robot poses from each individual camera.
   *
   * @return A list of Pairs containing the camera index and its estimated pose/timestamp.
   */
  public List<Pair<Integer, Pair<Pose3d, Double>>> getPerCameraEstimatedPoses() {
    List<Pair<Integer, Pair<Pose3d, Double>>> perCameraPoses = new ArrayList<>();
    for (int i = 0; i < photonPoseEstimators.size(); i++) {
      PhotonPoseEstimator estimator = photonPoseEstimators.get(i);
      PhotonCamera camera = CAMERAS[i];

      if (estimator != null && cameraEnabled[i] && camera != null && camera.isConnected()) {
        // Get the latest result directly from the camera first
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
          // Only update the estimator if we actually have targets
          var estimate = estimator.update();
          if (estimate.isPresent()) {
            EstimatedRobotPose pose = estimate.get();
            perCameraPoses.add(
                new Pair<>(i, new Pair<>(pose.estimatedPose, pose.timestampSeconds)));
          }
        }
      }
    }

    return perCameraPoses;
  }
}
