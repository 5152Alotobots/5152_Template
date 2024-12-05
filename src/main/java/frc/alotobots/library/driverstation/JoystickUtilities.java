package frc.alotobots.library.driverstation;

import lombok.experimental.UtilityClass;

/**
 * Utility class providing methods for processing joystick inputs.
 * Includes functions for deadband processing, scaling, and various input transformations
 * commonly used in robot control systems.
 */
@UtilityClass
public class JoystickUtilities {

  /**
   * Applies a deadband to a joystick input value and rescales the remaining range.
   * Values within the deadband range are set to zero, while values outside are
   * rescaled to maintain smooth control transitions.
   *
   * @param rawJoy The raw joystick input value (-1.0 to 1.0)
   * @param deadband The deadband range (0.0 to 1.0)
   * @return The processed joystick value with deadband applied
   */
  public static double joyDeadBnd(double rawJoy, double deadband) {
    double joy;
    if ((rawJoy < deadband) && (rawJoy > deadband * (-1))) {
      joy = 0;
    } else {
      joy = (rawJoy - (Math.abs(rawJoy) / rawJoy * deadband)) / (1 - deadband);
    }
    return joy;
  }

  /**
   * Squares the joystick input while preserving the sign.
   * This provides finer control at low speeds while maintaining full power availability.
   *
   * @param rawJoy The raw joystick input value (-1.0 to 1.0)
   * @return The squared joystick value, maintaining original sign
   */
  public static double joySqrd(double rawJoy) {
    double joy;
    joy = rawJoy * (Math.abs(rawJoy));
    return joy;
  }

  /**
   * Scales a joystick input by a constant factor.
   * Useful for limiting maximum speed or adjusting sensitivity.
   *
   * @param rawJoy The raw joystick input value (-1.0 to 1.0)
   * @param scalefactor The scaling factor to apply
   * @return The scaled joystick value
   */
  public static double joyScaled(double rawJoy, double scalefactor) {
    double joy;
    joy = rawJoy * scalefactor;
    return joy;
  }

  /**
   * Applies both deadband and squaring transformations to a joystick input.
   * Combines deadband filtering with squared response for enhanced control.
   *
   * @param rawJoy The raw joystick input value (-1.0 to 1.0)
   * @param deadband The deadband range (0.0 to 1.0)
   * @return The processed joystick value with deadband and squaring applied
   */
  public static double joyDeadBndSqrd(double rawJoy, double deadband) {
    double joy;
    joy = joySqrd(joyDeadBnd(rawJoy, deadband));
    return joy;
  }

  /**
   * Applies deadband and scaling transformations to a joystick input.
   * First applies deadband, then scales the result.
   *
   * @param rawJoy The raw joystick input value (-1.0 to 1.0)
   * @param deadband The deadband range (0.0 to 1.0)
   * @param scalefactor The scaling factor to apply after deadband
   * @return The processed joystick value with deadband and scaling applied
   */
  public static double joyDeadBndScaled(double rawJoy, double deadband, double scalefactor) {
    double joy;
    joy = joyScaled(joyDeadBnd(rawJoy, deadband), scalefactor);
    return joy;
  }

  /**
   * Applies deadband, squaring, and scaling transformations to a joystick input.
   * Processes input in order: deadband → squaring → scaling.
   *
   * @param rawJoy The raw joystick input value (-1.0 to 1.0)
   * @param deadband The deadband range (0.0 to 1.0)
   * @param scalefactor The scaling factor to apply after deadband and squaring
   * @return The fully processed joystick value
   */
  public static double joyDeadBndSqrdScaled(double rawJoy, double deadband, double scalefactor) {
    double joy;
    joy = joyScaled(joyDeadBndSqrd(rawJoy, deadband), scalefactor);
    return joy;
  }
}
