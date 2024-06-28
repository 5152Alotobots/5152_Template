package frc.alotobots.library.bling;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.networktables.GenericEntry;
import com.ctre.phoenix.led.Animation;
import frc.alotobots.Constants;

import java.awt.Color;
import java.util.Map;

/**
 * Handles telemetry for the Bling subsystem.
 */
public class BlingTelemetry {
    private final ShuffleboardTab blingTab;
    private final GenericEntry currentColorEntry;
    private final GenericEntry currentAnimationEntry;

    /**
     * Constructs a new BlingTelemetry object.
     */
    public BlingTelemetry() {
        this.blingTab = Shuffleboard.getTab("Bling");
        this.currentColorEntry = initializeShuffleboard();
        this.currentAnimationEntry = initializeAnimationEntry();
    }

    /**
     * Initializes the Shuffleboard layout for Bling telemetry.
     *
     * @return The GenericEntry for current color.
     */
    private GenericEntry initializeShuffleboard() {
        blingTab.addBoolean("Bling Enabled", () -> BlingSubsystemConstants.BLING_ENABLED)
            .withPosition(0, 0);

        return blingTab.add("Current Color", "None")
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();
    }

    /**
     * Initializes the Shuffleboard entry for current animation.
     *
     * @return The GenericEntry for current animation.
     */
    private GenericEntry initializeAnimationEntry() {
        return blingTab.add("Current Animation", "None")
            .withPosition(0, 2)
            .withSize(2, 1)
            .getEntry();
    }

    /**
     * Updates the Shuffleboard with the latest telemetry data.
     *
     * @param currentColor The current solid color of the LEDs.
     * @param currentAnimation The current animation running on the LEDs.
     */
    public void updateShuffleboard(Color currentColor, Animation currentAnimation) {
        currentColorEntry.setString(currentColor != null ? colorToString(currentColor) : "None");
        currentAnimationEntry.setString(currentAnimation != null ? currentAnimation.getClass().getSimpleName() : "None");
    }

    /**
     * Converts a Color object to a string representation.
     *
     * @param color The Color object to convert.
     * @return A string representation of the color in RGB format.
     */
    private String colorToString(Color color) {
        return String.format("RGB(%d, %d, %d)",
                color.getRed(),
                color.getGreen(),
                color.getBlue());
    }
}