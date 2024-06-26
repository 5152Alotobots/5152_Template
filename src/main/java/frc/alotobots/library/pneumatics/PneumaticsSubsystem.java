// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.alotobots.library.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.Constants;

public class PneumaticsSubsystem extends SubsystemBase {
  /** Creates a new SubSys_Pneumatics. */
  public PneumaticsSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  final Compressor pcmCompressor = new Compressor(Constants.Robot.CanId.PCM_CAN_ID, PneumaticsModuleType.CTREPCM);

  public void compressorOn() {
    pcmCompressor.enableDigital();
  }
}
