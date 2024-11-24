package frc.alotobots.library.bling;

import com.ctre.phoenix.led.*;
import java.awt.Color;
import lombok.experimental.UtilityClass;

/** Constants for the Bling subsystem. */
@UtilityClass
public class BlingSubsystemConstants {
  public static final boolean BLING_ENABLED = false;
  public static final double MAX_LED_BRIGHTNESS = 0.25;
  public static final int NUM_LEDS = 92;
  public static final int LED_OFFSET = 8;
  public static final boolean DISABLE_STATUS_LED = false;
  public static final CANdle.LEDStripType LED_TYPE = CANdle.LEDStripType.GRB;

  /** Predefined animations for the Bling subsystem. */
  @UtilityClass
  public static class Animations {
    public static final ColorFlowAnimation NO_ALLIANCE_ANIMATION =
        new ColorFlowAnimation(
            255, 0, 0, 0, 0.75, NUM_LEDS, ColorFlowAnimation.Direction.Forward, LED_OFFSET);
  }

  /** Predefined colors for the Bling subsystem. */
  @UtilityClass
  public static class Colors {
    public static final Color OFF_COLOR = Color.BLACK;
    public static final Color BLUE_ALLIANCE_COLOR = Color.BLUE;
    public static final Color RED_ALLIANCE_COLOR = Color.RED;
  }
}
