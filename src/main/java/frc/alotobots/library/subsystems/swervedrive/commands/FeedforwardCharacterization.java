//   ____  _ ____ ____       _    _        ___ _____ ___  ____     ___ _____ ____
//  | ___|/ | ___|___ \    / \  | |     / _ \_   _/ _ \| __) / _ \_   _/ ___|
//  |___ \| |___ \ __) | / _ \ | |    | | | || || | | |  _ \|| | || | \___ \
//   ___) | |___) / __/   / ___ \| |__  | |_| || || |_| | |_) | |_| || |  ___) |
//  |____/|_|____/_____| /_/   \_\_____\___/ |_| \___/|____/ \___/ |_| |____/
//
//
// 2025 ALOTOBOTS FRC 5152
// Robot Code
package frc.alotobots.library.subsystems.swervedrive.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

/**
 * Command that measures the velocity feedforward constants for the drive motors. This command
 * should only be used in voltage control mode.
 */
public class FeedforwardCharacterization extends Command {
  private static final double FF_START_DELAY = 2.0;
  private static final double FF_RAMP_RATE = 0.5;

  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private final Timer timer = new Timer();
  private final List<Double> velocitySamples = new LinkedList<>();
  private final List<Double> voltageSamples = new LinkedList<>();

  private double startTime;
  private boolean characterizationStarted = false;

  /**
   * Creates a new FeedforwardCharacterization command.
   *
   * @param swerveDriveSubsystem The subsystem to characterize
   */
  public FeedforwardCharacterization(SwerveDriveSubsystem swerveDriveSubsystem) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    addRequirements(swerveDriveSubsystem);
  }

  @Override
  public void initialize() {
    velocitySamples.clear();
    voltageSamples.clear();
    characterizationStarted = false;
    startTime = Timer.getFPGATimestamp();
    timer.restart();
  }

  @Override
  public void execute() {
    double currentTime = Timer.getFPGATimestamp();

    if (currentTime - startTime < FF_START_DELAY) {
      // Allow modules to orient
      swerveDriveSubsystem.runCharacterization(0.0);
    } else {
      if (!characterizationStarted) {
        characterizationStarted = true;
        timer.restart();
      }

      // Accelerate and gather data
      double voltage = timer.get() * FF_RAMP_RATE;
      swerveDriveSubsystem.runCharacterization(voltage);
      velocitySamples.add(swerveDriveSubsystem.getFFCharacterizationVelocity());
      voltageSamples.add(voltage);
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerveDriveSubsystem.runCharacterization(0.0);

    // Calculate and print results
    int n = velocitySamples.size();
    if (n > 0) {
      double sumX = 0.0;
      double sumY = 0.0;
      double sumXY = 0.0;
      double sumX2 = 0.0;
      for (int i = 0; i < n; i++) {
        sumX += velocitySamples.get(i);
        sumY += voltageSamples.get(i);
        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
      }
      double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
      double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

      NumberFormat formatter = new DecimalFormat("#0.00000");
      System.out.println("********** Drive FF Characterization Results **********");
      System.out.println("\tkS: " + formatter.format(kS));
      System.out.println("\tkV: " + formatter.format(kV));
    }
  }

  @Override
  public boolean isFinished() {
    return false; // Run until interrupted
  }
}
