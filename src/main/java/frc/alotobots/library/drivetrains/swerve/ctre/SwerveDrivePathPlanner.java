package frc.alotobots.library.drivetrains.swerve.ctre;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import frc.alotobots.library.drivetrains.swerve.ctre.mk4il32024.TunerConstants;

public class SwerveDrivePathPlanner {
  private final SwerveDriveSubsystem swerveDrive;
  private boolean flipPath;

  public SwerveDrivePathPlanner(SwerveDriveSubsystem swerveDrive) {
    this.swerveDrive = swerveDrive;
    configurePathPlanner();
  }

  private void configurePathPlanner() {
    double driveBaseRadius = 0;
    for (var moduleLocation : swerveDrive.getModuleLocations()) {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }

    flipPath =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configureHolonomic(
          swerveDrive::getPose,
          swerveDrive::seedFieldRelative,
          swerveDrive::getCurrentRobotChassisSpeeds,
          swerveDrive::setAutoRequest,
          new HolonomicPathFollowerConfig(
              new PIDConstants(10, 0, 0), // Translation PID
              new PIDConstants(7, 0, 0),  // Rotation PID
              TunerConstants.SPEED_AT_12_VOLTS_MPS,
              driveBaseRadius,
              new ReplanningConfig()),
          config,
          () -> flipPath,
          swerveDrive);
    } catch (Exception ex) {
      DriverStation.reportError("Failed to load PathPlanner config", ex.getStackTrace());
    }
  }

  public boolean getFlipPath() {
    return flipPath;
  }
}
