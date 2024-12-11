//   ____  _ ____ ____       _    _        ___ _____ ___  ____     ___ _____ ____
//  | ___|/ | ___|___ \    / \  | |     / _ \_   _/ _ \| __) / _ \_   _/ ___|
//  |___ \| |___ \ __) | / _ \ | |    | | | || || | | |  _ \|| | || | \___ \
//   ___) | |___) / __/   / ___ \| |__  | |_| || || |_| | |_) | |_| || |  ___) |
//  |____/|_|____/_____| /_/   \_\_____\___/ |_| \___/|____/ \___/ |_| |____/
//
//
// 2025 ALOTOBOTS FRC 5152
// Robot Code
package frc.alotobots.library.subsystems.swervedrive.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public interface TunerConstants {
  // Module Constants
  SwerveModuleConstants getFrontLeft();

  SwerveModuleConstants getFrontRight();

  SwerveModuleConstants getBackLeft();

  SwerveModuleConstants getBackRight();

  // Drivetrain Constants
  SwerveDrivetrainConstants getDrivetrainConstants();

  double getDriveBaseRadius();

  LinearVelocity getSpeedAt12Volts();

  LinearVelocity getTurtleSpeed();

  LinearVelocity getNominalSpeed();

  LinearVelocity getTurboSpeed();

  double getMaxModularRotationalRate();

  double getOdometryFrequency();

  // Physical Dimensions
  Distance getBumperLength();

  Distance getBumperWidth();

  // PathPlanner Integration
  RobotConfig getPathPlannerConfig();

  PathConstraints getPathfindingConstraints();

  PPHolonomicDriveController getHolonomicDriveController();

  // Control Gains
  Slot0Configs getSteerGains();

  Slot0Configs getDriveGains();

  // Module Translations
  Translation2d[] getModuleTranslations();
}
