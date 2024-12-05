package frc.alotobots.library.bling;

import com.ctre.phoenix.led.*;
import java.awt.Color;
import lombok.experimental.UtilityClass;

/**
 * Constants for configuring the robot's LED lighting (Bling) subsystem. Defines settings for LED
 * strips, animations, and colors.
 */
@UtilityClass
public class BlingSubsystemConstants {
  /** Global enable/disable flag for the Bling subsystem */
  public static final boolean BLING_ENABLED = false;

  /** Maximum brightness setting for LEDs (0.0-1.0) */
  public static final double MAX_LED_BRIGHTNESS = 0.25;

  /** Total number of LEDs in the strip */
  public static final int NUM_LEDS = 92;

  /** Number of LEDs to offset patterns by */
  public static final int LED_OFFSET = 8;

  /** Whether to disable the CANdle status LED */
  public static final boolean DISABLE_STATUS_LED = false;

  /** LED strip color channel ordering */
  public static final CANdle.LEDStripType LED_TYPE = CANdle.LEDStripType.GRB;

  /** Predefined animations for the Bling subsystem. */
  @UtilityClass
  /**
   * Predefined animation patterns for the LED strips. Contains constants for different light show
   * effects.
   */
  public static class Animations {
    /** Animation displayed when no alliance color is set */
    public static final ColorFlowAnimation NO_ALLIANCE_ANIMATION =
        new ColorFlowAnimation(
            255, 0, 0, 0, 0.75, NUM_LEDS, ColorFlowAnimation.Direction.Forward, LED_OFFSET);
  }

  /** Predefined colors for the Bling subsystem. */
  @UtilityClass
  /**
   * Standard colors used in LED patterns and animations. Defines common colors like alliance colors
   * and status indicators.
   */
  public static class Colors {
    /** Color used to turn LEDs off */
    public static final Color OFF_COLOR = Color.BLACK;

    /** Standard blue alliance color */
    public static final Color BLUE_ALLIANCE_COLOR = Color.BLUE;

    /** Standard red alliance color */
    public static final Color RED_ALLIANCE_COLOR = Color.RED;
  }
}
