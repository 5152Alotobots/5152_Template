package frc.alotobots.library.bling;

import static frc.alotobots.Constants.Robot.CanId.CANDLE_CAN_ID;
import static frc.alotobots.library.bling.BlingSubsystemConstants.*;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.util.Logger;
import java.awt.Color;
import lombok.Getter;

/**
 * Subsystem for controlling LED lighting (Bling) on the robot.
 *
 * <p>The Bling subsystem manages LED strips connected to a CTRE CANdle controller. It supports: -
 * Setting solid colors - Running animations - Queueing colors/animations - Alliance-based color
 * schemes - Brightness control
 *
 * <p>The subsystem uses a CANdle controller to drive addressable LED strips. LED updates happen in
 * the periodic() method.
 *
 * <p>Usage example:
 *
 * <pre>
 * BlingSubsystem bling = new BlingSubsystem();
 * // Set solid red color
 * bling.setSolidColor(Color.RED);
 * // Run rainbow animation
 * bling.runAnimation(new RainbowAnimation());
 * // Set to alliance color
 * bling.setLedToAllianceColor();
 * </pre>
 */
public class BlingSubsystem extends SubsystemBase {
  private final CANdle controller;
  private final BlingTelemetry telemetry;

  @Getter private Animation currentAnimation;
  @Getter private Animation queuedAnimation;
  @Getter private Color currentSolidColor;
  @Getter private Color queuedColor;

  /** Constructs a new Bling subsystem. */
  public BlingSubsystem() {
    Logger.info("Initializing BlingSubsystem");

    controller = new CANdle(CANDLE_CAN_ID);
    Logger.info("CANdle controller initialized");

    controller.configBrightnessScalar(MAX_LED_BRIGHTNESS);
    controller.configLEDType(LED_TYPE);
    controller.configStatusLedState(DISABLE_STATUS_LED);
    Logger.info("CANdle configuration completed");

    telemetry = new BlingTelemetry();
    Logger.info("Telemetry initialized");

    Logger.info("BlingSubsystem initialization completed");
  }

  /** Sets the LED strip to the color of the alliance reported by the FMS/DS. */
  public void setLedToAllianceColor() {
    clearAnimation();
    if (DriverStation.getAlliance().isPresent()) {
      setSolidColor(
          DriverStation.getAlliance().get() == DriverStation.Alliance.Red
              ? Colors.RED_ALLIANCE_COLOR
              : Colors.BLUE_ALLIANCE_COLOR);
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

  /** Clears the LEDs solid color (Turns off LEDs). */
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

  /** Sets the next color if available. If not, runs default behavior. */
  public void setQueuedColor() {
    if (queuedColor != null) {
      setSolidColor(queuedColor);
      queuedColor = null;
    } else {
      runDefault();
    }
  }

  /**
   * Sets the CANdle and attached LED's animation. Overrides previous solid colors and other
   * animations.
   *
   * @param animation The animation to set.
   */
  public void runAnimation(Animation animation) {
    clearAnimation();
    currentAnimation = animation;
  }

  /** Clears the current animation. */
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

  /** Runs the next animation if available. If not, runs default behavior. */
  public void runQueuedAnimation() {
    if (queuedAnimation != null) {
      runAnimation(queuedAnimation);
      queuedAnimation = null;
    } else {
      runDefault();
    }
  }

  /** Clears all settings (turns off LEDs). */
  public void clearAll() {
    clearAnimation();
    clearSolidColor();
  }

  /** Runs the default action. */
  public void runDefault() {
    setLedToAllianceColor();
  }

  /**
   * Updates the controller with the current state. Should be run in the periodic section of the
   * command.
   */
  public void update() {
    if (BLING_ENABLED) {
      if (currentAnimation == null && currentSolidColor != null) {
        controller.clearAnimation(0);
        controller.setLEDs(
            currentSolidColor.getRed(),
            currentSolidColor.getGreen(),
            currentSolidColor.getBlue(),
            0,
            LED_OFFSET,
            NUM_LEDS);
      } else if (currentAnimation != null) {
        controller.animate(currentAnimation, 0);
      }
    }
  }

  @Override
  public void periodic() {
    update();
    telemetry.updateShuffleboard(currentSolidColor, currentAnimation);
  }

  /**
   * Converts a Color object to a string representation.
   *
   * @param color The Color object to convert.
   * @return A string representation of the color in RGB format.
   */
  private String colorToString(Color color) {
    return String.format("RGB(%d, %d, %d)", color.getRed(), color.getGreen(), color.getBlue());
  }
}
