package frc.robot.library.bling;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

import java.awt.Color;

import static frc.robot.Constants.Robot.CanId.CANDLE_CAN_ID;
import static frc.robot.library.bling.BlingSubsystemConstants.*;

/**
 * Subsystem for controlling LED lighting on the robot.
 */
public class BlingSubsystem extends SubsystemBase {
    private final CANdle controller = new CANdle(CANDLE_CAN_ID);
    private final ShuffleboardTab blingTab = Shuffleboard.getTab("Bling");

    @Getter private Animation currentAnimation;
    @Getter private Animation queuedAnimation;
    @Getter private Color currentSolidColor;
    @Getter private Color queuedColor;

    /**
     * Constructs a new Bling subsystem.
     */
    public BlingSubsystem() {
        controller.configBrightnessScalar(MAX_LED_BRIGHTNESS);
        controller.configLEDType(LED_TYPE);
        controller.configStatusLedState(DISABLE_STATUS_LED);

        initializeShuffleboard();
    }

    /**
     * Initializes Shuffleboard displays for the Bling subsystem.
     */
    private void initializeShuffleboard() {
        blingTab.addBoolean("Bling Enabled", () -> BLING_ENABLED);
        blingTab.addString("Current Color", () -> currentSolidColor != null ? colorToString(currentSolidColor) : "None");
        blingTab.addString("Current Animation", () -> currentAnimation != null ? currentAnimation.getClass().getSimpleName() : "None");
    }

    /**
     * Sets the LED strip to the color of the alliance reported by the FMS/DS.
     */
    public void setLedToAllianceColor() {
        clearAnimation();
        if (DriverStation.getAlliance().isPresent()) {
            setSolidColor(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? Colors.RED_ALLIANCE_COLOR : Colors.BLUE_ALLIANCE_COLOR);
        } else {
            runAnimation(Animations.NO_ALLIANCE_ANIMATION);
        }
    }

    /**
     * Sets the color of the LED strip. Overrides previously running animations.
     *
     * @param color The color to set the LEDs to.
     */
    public void setSolidColor(Color color) {
        clearAnimation();
        currentSolidColor = color;
    }

    /**
     * Clears the LEDs solid color (Turns off LEDs).
     */
    public void clearSolidColor() {
        currentSolidColor = Colors.OFF_COLOR;
    }

    /**
     * Queues a color and doesn't set the next one until released.
     *
     * @param toQueue The color to queue.
     */
    public void queueColor(Color toQueue) {
        queuedColor = toQueue;
    }

    /**
     * Sets the next color if available. If not, runs default behavior.
     */
    public void setQueuedColor() {
        if (queuedColor != null) {
            setSolidColor(queuedColor);
            queuedColor = null;
        } else {
            runDefault();
        }
    }

    /**
     * Sets the CANdle and attached LED's animation. Overrides previous solid colors and other animations.
     *
     * @param animation The animation to set.
     */
    public void runAnimation(Animation animation) {
        clearAnimation();
        currentAnimation = animation;
    }

    /**
     * Clears the current animation.
     */
    public void clearAnimation() {
        controller.clearAnimation(0);
        currentAnimation = null;
    }

    /**
     * Queues an animation and doesn't run the next one until released.
     *
     * @param toQueue The animation to queue.
     */
    public void queueAnimation(Animation toQueue) {
        queuedAnimation = toQueue;
    }

    /**
     * Runs the next animation if available. If not, runs default behavior.
     */
    public void runQueuedAnimation() {
        if (queuedAnimation != null) {
            runAnimation(queuedAnimation);
            queuedAnimation = null;
        } else {
            runDefault();
        }
    }

    /**
     * Clears all settings (turns off LEDs).
     */
    public void clearAll() {
        clearAnimation();
        clearSolidColor();
    }

    /**
     * Runs the default action.
     */
    public void runDefault() {
        setLedToAllianceColor();
    }

    /**
     * Updates the controller with the current state.
     * Should be run in the periodic section of the command.
     */
    public void update() {
        if (BLING_ENABLED) {
            if (currentAnimation == null && currentSolidColor != null) {
                controller.clearAnimation(0);
                controller.setLEDs(
                        currentSolidColor.getRed(),
                        currentSolidColor.getGreen(),
                        currentSolidColor.getBlue(),
                        0, LED_OFFSET, NUM_LEDS
                );
            } else if (currentAnimation != null) {
                controller.animate(currentAnimation, 0);
            }
        }
    }

    @Override
    public void periodic() {
        update();
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