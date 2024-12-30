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
package frc.alotobots.library.subsystems.vision.questnav.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class OculusConstants {
  public static final Transform2d OCULUS_TO_ROBOT =
      new Transform2d(
          0.16, 0, Rotation2d.fromDegrees(0)); // X: +Forwards, Y: + Left, ROT: + CCW .16 x

  public static final double RESET_TIMEOUT_SECONDS = 0.2;
  public static final int MAX_RESET_ATTEMPTS = 3;
}
