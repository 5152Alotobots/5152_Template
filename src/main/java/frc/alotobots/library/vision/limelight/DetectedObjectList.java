package frc.alotobots.library.vision.limelight;

import static frc.alotobots.library.vision.limelight.LimelightSubsystemConstants.*;

import edu.wpi.first.math.geometry.*;
import java.util.ArrayList;
import java.util.Comparator;
import lombok.AllArgsConstructor;
import lombok.Data;

/**
 * A specialized ArrayList to store and manage DetectedObjectPair objects. This list automatically
 * decreases the confidence value of each entry and removes entries with zero confidence.
 */
public class DetectedObjectList extends ArrayList<DetectedObjectList.DetectedObjectPair> {

  /**
   * Adds a new DetectedObjectPair to the list or updates an existing one if within tolerance.
   *
   * @param pair The DetectedObjectPair to be added or updated.
   * @return true if the list changed as a result of the call, false otherwise.
   */
  @Override
  public boolean add(DetectedObjectPair pair) {
    for (DetectedObjectPair existingPair : this) {
      if (isPoseWithinTolerance(
          existingPair.getObject().getPose(), pair.getObject().getPose(), POSE_TOLERANCE)) {
        existingPair.setObject(pair.getObject());
        existingPair.setConfidence(pair.getConfidence());
        return true;
      }
    }
    return super.add(pair);
  }

  /** Updates the list by decreasing confidence values and removing low-confidence entries. */
  public void update() {
    removeIf(
        pair -> {
          double updatedConfidence = pair.getConfidence() - CONFIDENCE_DECAY;
          if (updatedConfidence <= 0.0) {
            return true;
          } else {
            pair.setConfidence(updatedConfidence);
            return false;
          }
        });
  }

  /**
   * Sorts the list by confidence values in descending order.
   *
   * @return The sorted list.
   */
  public DetectedObjectList sortByConfidence() {
    sort(Comparator.comparingDouble(DetectedObjectPair::getConfidence).reversed());
    return this;
  }

  /**
   * Sorts the list by distance from a given pose.
   *
   * @param pose The reference Pose2d to calculate distances from.
   * @return The sorted list.
   */
  public DetectedObjectList sortByPose(Pose2d pose) {
    sort(
        Comparator.comparingDouble(
            pair -> {
              Translation2d translation =
                  new Translation2d(
                      pair.getObject().getPose().getX(), pair.getObject().getPose().getY());
              return pose.getTranslation().getDistance(translation);
            }));
    return this;
  }

  /**
   * Checks if two Pose3d objects are within a specified tolerance range.
   *
   * @param pose1 The first Pose3d object.
   * @param pose2 The second Pose3d object.
   * @param tolerance The tolerance range in meters.
   * @return true if the poses are within the tolerance range, false otherwise.
   */
  private boolean isPoseWithinTolerance(Pose3d pose1, Pose3d pose2, double tolerance) {
    Translation3d translation1 = pose1.getTranslation();
    Translation3d translation2 = pose2.getTranslation();

    double deltaX = Math.abs(translation1.getX() - translation2.getX());
    double deltaY = Math.abs(translation1.getY() - translation2.getY());
    double deltaZ = Math.abs(translation1.getZ() - translation2.getZ());

    return deltaX <= tolerance && deltaY <= tolerance && deltaZ <= tolerance;
  }

  /** Represents a pair of a DetectedObject and its associated confidence value. */
  @Data
  @AllArgsConstructor
  public static class DetectedObjectPair {
    private DetectedObject object;
    private double confidence;

    @Override
    public String toString() {
      return String.format(
          "(X: %.2f, Y: %.2f, Z: %.2f, FieldAngle: %.2f) Class: %S, Conf: %.2f",
          object.getPose().getX(),
          object.getPose().getY(),
          object.getPose().getZ(),
          object.getAngle(),
          object.getType(),
          confidence);
    }
  }

  /**
   * Converts the list to an array of strings representing each DetectedObjectPair.
   *
   * @return An array of strings representing the DetectedObjectPairs.
   */
  @Override
  public String[] toArray() {
    return stream().map(DetectedObjectPair::toString).toArray(String[]::new);
  }
}
