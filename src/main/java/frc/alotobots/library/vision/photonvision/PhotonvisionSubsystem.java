package frc.alotobots.library.vision.photonvision;

import static frc.alotobots.library.vision.photonvision.PhotonvisionSubsystemConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

/**
 * A subsystem that manages multiple PhotonVision cameras and provides pose estimation
 * functionality.
 */
public class PhotonvisionSubsystem extends SubsystemBase {
  private final AprilTagFieldLayout aprilTagFieldLayout;
  private ArrayList<PhotonPoseEstimator> photonPoseEstimators;
  private PhotonvisionTelemetry telemetry;

  /** Constructs a new PhotonvisionSubsystem. */
  public PhotonvisionSubsystem() {
    System.out.println("Initializing PhotonvisionSubsystem");
    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    initializePoseEstimators();
    telemetry = new PhotonvisionTelemetry();
    System.out.println("PhotonvisionSubsystem initialized");
  }

  private void initializePoseEstimators() {
    if (photonPoseEstimators == null) {
      System.out.println("Initializing PhotonPoseEstimators");
      photonPoseEstimators = new ArrayList<>();

      if (CAMERAS.length != CAMERA_OFFSETS.length) {
        throw new RuntimeException(
            "PhotonCamera object is missing offset! Did you add an offset in Photonvision_Constants?");
      }
      for (int i = 0; i < CAMERAS.length; i++) {
        PhotonPoseEstimator estimator =
            new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                CAMERAS[i],
                CAMERA_OFFSETS[i]);
        estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        photonPoseEstimators.add(estimator);
      }
      System.out.println("PhotonPoseEstimators initialized");
    }
  }

  @Override
  public void periodic() {
    if (USE_VISION_POSE_ESTIMATION) {
      Optional<Pair<Pose2d, Double>> estimatedPose = getEstimatedVisionPose2d();
      telemetry.updateShuffleboard(estimatedPose.map(Pair::getFirst));
    }
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
    for (PhotonPoseEstimator estimator : photonPoseEstimators) {
      estimator.setLastPose(previousPose);
      estimator.update().ifPresent(estimates::add);
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
    for (PhotonPoseEstimator estimator : photonPoseEstimators) {
      estimator.update().ifPresent(estimates::add);
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

  /**
   * Averages the estimated robot poses from multiple cameras.
   *
   * @param estimates An ArrayList of EstimatedRobotPose objects.
   * @return A Pair containing the average Pose3d and timestamp, or an empty Optional if the list is
   *     empty.
   */
  private Optional<Pair<Pose3d, Double>> averageEstimates(ArrayList<EstimatedRobotPose> estimates) {
    if (estimates.isEmpty()) {
      return Optional.empty();
    }

    Pose3d averagePose = estimates.get(0).estimatedPose;
    double timestamp = estimates.get(0).timestampSeconds;
    for (int i = 1; i < estimates.size(); i++) {
      averagePose = averagePose.interpolate(estimates.get(i).estimatedPose, 1.0 / (i + 1));
      timestamp += estimates.get(i).timestampSeconds;
    }
    timestamp /= estimates.size();

    return Optional.of(new Pair<>(averagePose, timestamp));
  }
}
