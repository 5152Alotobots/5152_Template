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
package frc.alotobots.library.subsystems.bling.io;

import com.ctre.phoenix.led.Animation;
import org.littletonrobotics.junction.AutoLog;

public interface BlingIO {
  @AutoLog
  public static class BlingIOInputs {
    public LoggedColor currentSolidColor;
    public boolean hasAnimation = false;
    public boolean hasColor = false;
    public String animationName = "";
  }

  public static record LoggedColor(int red, int green, int blue) {}

  public default void updateInputs(BlingIOInputs inputs) {}

  public default void setAnimation(Animation animation) {}

  public default void clearAnimation() {}

  public default void setSolidColor(BlingIO.LoggedColor color) {}

  public default void clearSolidColor() {}
}
