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
package frc.alotobots;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OI {
  public static final double DEADBAND = 0.1;
  private static final CommandXboxController driverController = new CommandXboxController(0);

  // Driver controller inputs for translation and rotation
  public static double getTranslateForwardAxis() {
    return driverController.getLeftY();
  }

  public static double getTranslateStrafeAxis() {
    return driverController.getLeftX();
  }

  public static double getRotationAxis() {
    return driverController.getRightX();
  }

  // Speed control triggers
  public static double getTurtleSpeedTrigger() {
    return driverController.getLeftTriggerAxis();
  }

  public static double getTurboSpeedTrigger() {
    return driverController.getRightTriggerAxis();
  }

  // Buttons
  public static Trigger driveFacingBestObjectButton = driverController.a();
  // Add any additional controller bindings or button methods here
}
