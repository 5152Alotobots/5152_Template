package frc.alotobots.library.pneumatics;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Compressor;
import frc.alotobots.Constants;

import java.util.Map;

/**
 * Handles telemetry for the Pneumatics subsystem.
 */
public class PneumaticsTelemetry {
    private final ShuffleboardTab pneumaticsTab;
    private final GenericEntry compressorStateEntry;
    private final GenericEntry pressureSwitchEntry;
    private final GenericEntry currentEntry;

    /**
     * Constructs a new PneumaticsTelemetry object.
     */
    public PneumaticsTelemetry() {
        this.pneumaticsTab = Shuffleboard.getTab("Pneumatics");
        this.compressorStateEntry = initializeShuffleboard();
        this.pressureSwitchEntry = initializePressureSwitchEntry();
        this.currentEntry = initializeCurrentEntry();
    }

    /**
     * Initializes the Shuffleboard layout for Pneumatics telemetry.
     *
     * @return The GenericEntry for compressor state.
     */
    private GenericEntry initializeShuffleboard() {
        return pneumaticsTab.add("Compressor State", "Unknown")
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();
    }

    /**
     * Initializes the Shuffleboard entry for pressure switch state.
     *
     * @return The GenericEntry for pressure switch state.
     */
    private GenericEntry initializePressureSwitchEntry() {
        return pneumaticsTab.add("Pressure Switch", false)
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();
    }

    /**
     * Initializes the Shuffleboard entry for compressor current.
     *
     * @return The GenericEntry for compressor current.
     */
    private GenericEntry initializeCurrentEntry() {
        return pneumaticsTab.add("Compressor Current", 0.0)
            .withPosition(0, 2)
            .withSize(2, 1)
            .getEntry();
    }

    /**
     * Updates the Shuffleboard with the latest telemetry data.
     *
     * @param compressor The Compressor object to get data from.
     */
    public void updateShuffleboard(Compressor compressor) {
        compressorStateEntry.setString(compressor.isEnabled() ? "Enabled" : "Disabled");
        pressureSwitchEntry.setBoolean(compressor.getPressureSwitchValue());
        currentEntry.setDouble(compressor.getCurrent());
    }
}