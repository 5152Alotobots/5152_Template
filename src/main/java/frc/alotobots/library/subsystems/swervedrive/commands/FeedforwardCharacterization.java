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
import edu.wpi.first.wpilibj2.command.Commands;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

public class FeedforwardCharacterization {
    private static final double FF_START_DELAY = 2.0;
    private static final double FF_RAMP_RATE = 0.5;

    /**
     * Creates a command that measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     */
    public static Command create(SwerveDriveSubsystem swerveDriveSubsystem) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
            // Reset data
            Commands.runOnce(
                () -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

            // Allow modules to orient
            Commands.run(
                    () -> {
                        swerveDriveSubsystem.runCharacterization(0.0);
                    },
                    swerveDriveSubsystem)
                .withTimeout(FF_START_DELAY),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(
                    () -> {
                        double voltage = timer.get() * FF_RAMP_RATE;
                        swerveDriveSubsystem.runCharacterization(voltage);
                        velocitySamples.add(swerveDriveSubsystem.getFFCharacterizationVelocity());
                        voltageSamples.add(voltage);
                    },
                    swerveDriveSubsystem)

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                        int n = velocitySamples.size();
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
                    }));
    }
}
