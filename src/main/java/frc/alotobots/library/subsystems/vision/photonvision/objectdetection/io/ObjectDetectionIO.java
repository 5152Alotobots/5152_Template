/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2024 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io;

import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for a single camera's object detection IO operations. */
public interface ObjectDetectionIO {
  @AutoLog
  public static class ObjectDetectionIOInputs {
    public boolean connected = false;
    public DetectedObject[] detectedObjects = new DetectedObject[0];
  }

  public static record DetectedObject(
      double timestamp, Transform3d targetToRobot, double confidence, int classId) {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ObjectDetectionIOInputs inputs) {}
}
