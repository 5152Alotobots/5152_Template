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

        flipPath = DriverStation.getAlliance().isPresent() &&
                DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        AutoBuilder.configureHolonomic(
                swerveDrive::getPose,
                swerveDrive::seedFieldRelative,
                swerveDrive::getCurrentRobotChassisSpeeds,
                swerveDrive::setAutoRequest,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        TunerConstants.SPEED_AT_12_VOLTS_MPS,
                        driveBaseRadius,
                        new ReplanningConfig()
                ),
                () -> flipPath,
                swerveDrive
        );
    }

    public boolean getFlipPath() {
        return flipPath;
    }
}
