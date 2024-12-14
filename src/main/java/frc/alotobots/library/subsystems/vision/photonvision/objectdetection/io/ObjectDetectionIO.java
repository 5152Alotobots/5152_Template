//   ____  _ ____ ____       _    _        ___ _____ ___  ____     ___ _____ ____
//  | ___|/ | ___|___ \    / \  | |     / _ \_   _/ _ \| __) / _ \_   _/ ___|
//  |___ \| |___ \ __) | / _ \ | |    | | | || || | | |  _ \|| | || | \___ \
//   ___) | |___) / __/   / ___ \| |__  | |_| || || |_| | |_) | |_| || |  ___) |
//  |____/|_|____/_____| /_/   \_\_____\___/ |_| \___/|____/ \___/ |_| |____/
//
//
// 2025 ALOTOBOTS FRC 5152
// Robot Code
package frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io;

import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for a single camera's object detection IO operations. */
public interface ObjectDetectionIO {
  @AutoLog
  public static class ObjectDetectionInputs {
    public double timestamp = 0.0;
    public boolean hasTargets = false;
    public double[] targetYaws = new double[0];
    public double[] targetPitches = new double[0];
    public int[] targetClassIds = new int[0];
    public double[] targetAreas = new double[0];
    public Transform3d cameraToRobot = new Transform3d();
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ObjectDetectionInputs inputs) {}
}
