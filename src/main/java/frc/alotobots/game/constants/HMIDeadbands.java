package frc.alotobots.game.constants;

public final class HMIDeadbands {
  private HMIDeadbands() {} // Prevent instantiation

  // Basic drive control deadbands
  public static final double DRIVER_FWD_AXIS_DEADBAND = 0.1;
  public static final double DRIVER_STR_AXIS_DEADBAND = 0.1;
  public static final double DRIVER_ROT_AXIS_DEADBAND = 0.1;
  public static final double TRIGGER_DEADBAND = 0.3;

  /* Example game-specific deadbands:
   * public static final double PRIMARY_MECHANISM_AXIS_DEADBAND = 0.1;
   * public static final double SECONDARY_MECHANISM_AXIS_DEADBAND = 0.15;
   * public static final double TERTIARY_MECHANISM_AXIS_DEADBAND = 0.2;
   */
}
