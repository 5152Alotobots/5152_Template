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
package frc.alotobots.library.subsystems.vision.questnav.io;

import org.littletonrobotics.junction.AutoLog;

public interface OculusIO {
  @AutoLog
  public static class OculusIOInputs {
    public float[] position = new float[] {0.0f, 0.0f, 0.0f};
    public float[] quaternion = new float[] {0.0f, 0.0f, 0.0f, 0.0f};
    public float[] eulerAngles = new float[] {0.0f, 0.0f, 0.0f};
    public double timestamp = 0.0;
    public int frameCount = 0;
    public double batteryPercent = 0.0;
    public int misoValue = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(OculusIOInputs inputs) {}

  /** Sets MOSI value for Quest communication */
  public default void setMosi(int value) {}

  /** Sets the pose components for pose reset */
  public default void setResetPose(double x, double y, double rotation) {}
}
