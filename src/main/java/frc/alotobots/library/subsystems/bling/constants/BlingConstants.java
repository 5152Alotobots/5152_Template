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
package frc.alotobots.library.subsystems.bling.constants;

import com.ctre.phoenix.led.*;
import frc.alotobots.library.subsystems.bling.io.BlingIO;

public class BlingConstants {
  public static final double MAX_LED_BRIGHTNESS = .25;
  public static final int NUM_LEDS = 92;
  public static final int LED_OFFSET = 8;
  public static final boolean DISABLE_STATUS_LED = false;
  public static final CANdle.LEDStripType LED_TYPE = CANdle.LEDStripType.GRB;

  public static final class Animations {
    public static final ColorFlowAnimation SHOOTING_ANIMATION =
        new ColorFlowAnimation(
            255, 145, 0, 0, 1, NUM_LEDS, ColorFlowAnimation.Direction.Forward, LED_OFFSET);
    public static final ColorFlowAnimation NO_ALLIANCE_ANIMATION =
        new ColorFlowAnimation(
            255, 0, 0, 0, 0.75, NUM_LEDS, ColorFlowAnimation.Direction.Forward, LED_OFFSET);
  }

  public static final class Colors {
    public static final BlingIO.LoggedColor OFF_COLOR = new BlingIO.LoggedColor(0, 0, 0);
    public static final BlingIO.LoggedColor BLUE_ALLIANCE_COLOR =
        new BlingIO.LoggedColor(0, 0, 255);
    public static final BlingIO.LoggedColor RED_ALLIANCE_COLOR = new BlingIO.LoggedColor(255, 0, 0);
    public static final BlingIO.LoggedColor NO_ALLIANCE_COLOR =
        new BlingIO.LoggedColor(255, 255, 0);
    public static final BlingIO.LoggedColor INTAKE_OCCUPIED_COLOR =
        new BlingIO.LoggedColor(0, 255, 0);
    public static final BlingIO.LoggedColor SHOOTER_OCCUPIED_COLOR =
        new BlingIO.LoggedColor(140, 48, 255);
    public static final BlingIO.LoggedColor SHOOTER_READY_COLOR =
        new BlingIO.LoggedColor(255, 145, 0);
  }
}
