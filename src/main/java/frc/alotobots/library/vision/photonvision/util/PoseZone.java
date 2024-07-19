package frc.alotobots.library.vision.photonvision.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public record PoseZone(Pose2d lowerLeftBound, Pose2d upperRightBound) {

  /**
   * @param pose The {@link Pose3d} reference
   * @return If the pose is inside the zone
   */
  public boolean isInside(Pose3d pose) {
    Pose2d lowerLeft = lowerLeftBound();
    Pose2d upperRight = upperRightBound();
    boolean xInside = pose.getX() > lowerLeft.getX() && pose.getX() < upperRight.getX();
    boolean yInside = pose.getY() > lowerLeft.getY() && pose.getY() < upperRight.getY();
    return xInside && yInside;
  }

  public boolean isInside(Pose2d pose) {
    Pose2d lowerLeft = lowerLeftBound();
    Pose2d upperRight = upperRightBound();
    boolean xInside = pose.getX() > lowerLeft.getX() && pose.getX() < upperRight.getX();
    boolean yInside = pose.getY() > lowerLeft.getY() && pose.getY() < upperRight.getY();
    return xInside && yInside;
  }
}
