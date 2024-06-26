package frc.alotobots.library.vision.photonvision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.ArrayList;
import java.util.Optional;

import static frc.alotobots.library.vision.photonvision.PhotonvisionSubsystemConstants.*;

/**
 * A subsystem that manages multiple PhotonVision cameras and provides pose estimation functionality.
 */
public class PhotonvisionSubsystem implements Subsystem {
  private final AprilTagFieldLayout aprilTagFieldLayout;
  private ArrayList<PhotonPoseEstimator> photonPoseEstimators;

  /**
   * Constructs a new SubSys_Photonvision with the given PhotonCamera objects.
   */
  public PhotonvisionSubsystem() {
    System.out.println("Initializing PhotonvisionSubsystem");
    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    initializePoseEstimators();
    System.out.println("PhotonvisionSubsystem initialized");
  }

  private void initializePoseEstimators() {
    if (photonPoseEstimators == null) {
      System.out.println("Initializing PhotonPoseEstimators");
      photonPoseEstimators = new ArrayList<>();

      // Ensure that we have offsets for each camera object
      if (CAMERAS.length != CAMERA_OFFSETS.length) {
        throw new RuntimeException("PhotonCamera object is missing offset! Did you add an offset in Photonvision_Constants?");
      }
      // Loop, creating pose estimators
      for (int i = 0; i < CAMERAS.length; i++) {
        PhotonPoseEstimator estimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CAMERAS[i], CAMERA_OFFSETS[i]);
        estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_LAST_POSE);
        photonPoseEstimators.add(estimator);
      }
      System.out.println("PhotonPoseEstimators initialized");
    }
  }

  @Override
  public void periodic() {
    if (USE_VISION_POSE_ESTIMATION) {
      Optional<Pair<Pose2d, Double>> estimatedPose = getEstimatedVisionPose2d();
      if (estimatedPose.isPresent()) {
        SmartDashboard.putString("Vision/Vision Pose Estimate", estimatedPose.get().getFirst().toString());
      } else {
        SmartDashboard.putString("Vision/Vision Pose Estimate", "NONE");
      }
    }
  }

  /**
   * Returns the estimated 3D robot pose using the MultiTag pose from the coprocessor, if available.
   *
   * @param previousPose The previous pose of the robot.
   * @return A Pair containing the estimated Pose3d and the timestamp of the estimation, or an empty Optional if the pose is not available.
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
   * @return A Pair containing the estimated Pose3d and the timestamp of the estimation, or an empty Optional if the pose is not available.
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
   * @return A Pair containing the estimated Pose2d and the timestamp of the estimation, or an empty Optional if the pose is not available.
   */
  public Optional<Pair<Pose2d, Double>> getEstimatedVisionPose2d(Pose2d previousPose) {
    return getEstimatedVisionPose3d(previousPose).map(pair -> new Pair<>(pair.getFirst().toPose2d(), pair.getSecond()));
  }

  /**
   * Returns the estimated 2D robot pose using the MultiTag pose from the coprocessor, if available.
   *
   * @return A Pair containing the estimated Pose2d and the timestamp of the estimation, or an empty Optional if the pose is not available.
   */
  public Optional<Pair<Pose2d, Double>> getEstimatedVisionPose2d() {
    return getEstimatedVisionPose3d().map(pair -> new Pair<>(pair.getFirst().toPose2d(), pair.getSecond()));
  }

  /**
   * Averages the estimated robot poses from multiple cameras.
   *
   * @param estimates An ArrayList of EstimatedRobotPose objects.
   * @return A Pair containing the average Pose3d and timestamp, or an empty Optional if the list is empty.
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