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

/**
 * Constants used by the Oculus Quest navigation subsystem. Contains configuration values for
 * physical setup and operation parameters.
 */
public class OculusConstants {
  /**
   * Transform from the robot center to the headset. Coordinate system: - X:
   * Positive is forwards - Y: Positive is left - Rotation: Positive is counter-clockwise
   */
  public static final Transform2d ROBOT_TO_OCULUS = new Transform2d(0.075, 0.0, new Rotation2d());

  /**
   * Timeout duration in seconds for reset operations (pose reset, heading reset, ping). If a reset
   * operation takes longer than this time, it will be considered failed.
   */
  public static final double RESET_TIMEOUT_SECONDS = 0.2;

  /**
   * Maximum number of attempts for reset operations. If a reset operation fails this many times,
   * the command will terminate.
   */
  public static final int MAX_RESET_ATTEMPTS = 3;
}
