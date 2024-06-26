package frc.robot.library.driverstation;

import lombok.experimental.UtilityClass;

/** Add your docs here. */
@UtilityClass
public class JoystickUtilities {

  public static double joyDeadBnd(double rawJoy, double deadband) {
    double joy;
    if ((rawJoy < deadband) && (rawJoy > deadband * (-1))) {
      joy = 0;
    } else {
      joy = (rawJoy - (Math.abs(rawJoy) / rawJoy * deadband)) / (1 - deadband);
    }
    return joy;
  }

  public static double joySqrd(double rawJoy) {
    double joy;
    joy = rawJoy * (Math.abs(rawJoy));
    return joy;
  }

  public static double joyScaled(double rawJoy, double scalefactor) {
    double joy;
    joy = rawJoy * scalefactor;
    return joy;
  }

  public static double joyDeadBndSqrd(double rawJoy, double deadband) {
    double joy;
    joy = joySqrd(joyDeadBnd(rawJoy, deadband));
    return joy;
  }

  public static double joyDeadBndScaled(double rawJoy, double deadband, double scalefactor) {
    double joy;
    joy = joyScaled(joyDeadBnd(rawJoy, deadband), scalefactor);
    return joy;
  }

  public static double joyDeadBndSqrdScaled(double rawJoy, double deadband, double scalefactor) {
    double joy;
    joy = joyScaled(joyDeadBndSqrd(rawJoy, deadband), scalefactor);
    return joy;
  }
}