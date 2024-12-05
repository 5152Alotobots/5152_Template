package frc.alotobots.util;

import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Utility class for logging messages at different severity levels.
 * Wraps WPILib's DataLogManager to provide standard logging levels.
 */
public class Logger {
  /**
   * Log an informational message.
   *
   * @param message The message to log
   */
  public static void info(String message) {
    DataLogManager.log("[INFO] " + message);
  }

  /**
   * Log a warning message.
   *
   * @param message The warning message to log
   */
  public static void warn(String message) {
    DataLogManager.log("[WARN] " + message);
  }

  /**
   * Log an error message.
   *
   * @param message The error message to log
   */
  public static void error(String message) {
    DataLogManager.log("[ERROR] " + message);
  }

  /**
   * Log a debug message.
   *
   * @param message The debug message to log
   */
  public static void debug(String message) {
    DataLogManager.log("[DEBUG] " + message);
  }
}
