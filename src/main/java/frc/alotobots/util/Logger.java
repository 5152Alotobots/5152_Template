package frc.alotobots.util;

import edu.wpi.first.wpilibj.DataLogManager;

public class Logger {
  public static void info(String message) {
    DataLogManager.log("[INFO] " + message);
  }

  public static void warn(String message) {
    DataLogManager.log("[WARN] " + message);
  }

  public static void error(String message) {
    DataLogManager.log("[ERROR] " + message);
  }

  public static void debug(String message) {
    DataLogManager.log("[DEBUG] " + message);
  }
}
