package frc.alotobots.library.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.Constants;

/** Subsystem for controlling the pneumatics system on the robot. */
public class PneumaticsSubsystem extends SubsystemBase {
  private final Compressor compressor;
  private final PneumaticsTelemetry telemetry;

  /** Constructs a new PneumaticsSubsystem. */
  public PneumaticsSubsystem() {
    System.out.println("Initializing PneumaticsSubsystem");

    compressor = new Compressor(Constants.Robot.CanId.PCM_CAN_ID, PneumaticsModuleType.CTREPCM);
    telemetry = new PneumaticsTelemetry();

    System.out.println("PneumaticsSubsystem initialized");
  }

  @Override
  public void periodic() {
    telemetry.updateShuffleboard(compressor);
  }

  /** Enables the compressor in digital mode. */
  public void compressorOn() {
    compressor.enableDigital();
  }

  /** Disables the compressor. */
  public void compressorOff() {
    compressor.disable();
  }

  /**
   * Checks if the compressor is currently running.
   *
   * @return true if the compressor is running, false otherwise.
   */
  public boolean isCompressorRunning() {
    return compressor.isEnabled();
  }

  /**
   * Gets the current draw of the compressor.
   *
   * @return The current draw in amps.
   */
  public double getCompressorCurrent() {
    return compressor.getCurrent();
  }

  /**
   * Gets the state of the pressure switch.
   *
   * @return true if the pressure switch is triggered (system is at pressure), false otherwise.
   */
  public boolean getPressureSwitchValue() {
    return compressor.getPressureSwitchValue();
  }
}
