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
package frc.alotobots.library.subsystems.bling;

import com.ctre.phoenix.led.Animation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.bling.io.BlingIO;
import frc.alotobots.library.subsystems.bling.io.BlingIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class BlingSubsystem extends SubsystemBase {
  private final BlingIO io;
  private final BlingIOInputsAutoLogged inputs = new BlingIOInputsAutoLogged();

  public BlingSubsystem(BlingIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update inputs
    io.updateInputs(inputs);
    Logger.processInputs("Bling", inputs);
    if (this.getCurrentCommand() != null) {
      Logger.recordOutput("Bling/CurrentCommand", this.getCurrentCommand().getName());
    } else {
      Logger.recordOutput("Bling/CurrentCommand", "None");
    }
  }

  public void setSolidColor(BlingIO.LoggedColor color) {
    io.setSolidColor(color);
  }

  public void setAnimation(Animation animation) {
    io.setAnimation(animation);
  }

  public void clear() {
    io.clearAnimation();
    io.clearSolidColor();
  }
}
