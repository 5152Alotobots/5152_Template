package frc.alotobots.util;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

/**
 * Utility class for Phoenix 6 configuration and status handling.
 */
public final class PhoenixUtil {
  private PhoenixUtil() {
    throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
  }

  /**
   * Attempts to run the command until no error is produced or max attempts reached.
   *
   * @param maxAttempts Maximum number of attempts to try the command
   * @param command The command to execute that returns a StatusCode
   * @return True if command succeeded, false if max attempts reached without success
   */
  public static boolean tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) {
        return true;
      }
    }
    return false;
  }

  /**
   * Checks if a Phoenix 6 status code indicates success.
   * 
   * @param status The status code to check
   * @return True if status indicates success, false otherwise
   */
  public static boolean isOK(StatusCode status) {
    return status.isOK();
  }

  /**
   * Checks if a Phoenix 6 status code indicates an error.
   *
   * @param status The status code to check
   * @return True if status indicates error, false otherwise
   */
  public static boolean isError(StatusCode status) {
    return !status.isOK(); 
  }
}
package frc.alotobots.util;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

/** Utility class for Phoenix 6 configuration and status handling. */
public final class PhoenixUtil {
  private PhoenixUtil() {
    throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
  }

  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }
}
