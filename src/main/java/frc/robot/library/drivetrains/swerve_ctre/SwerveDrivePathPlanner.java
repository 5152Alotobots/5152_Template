package frc.robot.library.drivetrains.swerve_ctre;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.library.drivetrains.swerve_ctre.mk4il32024.TunerConstants_MK4iL3_2024;

public class SwerveDrivePathPlanner {
    private final SubSys_SwerveDrive swerveDrive;
    private boolean flipPath;

    public SwerveDrivePathPlanner(SubSys_SwerveDrive swerveDrive) {
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
                        TunerConstants_MK4iL3_2024.SPEED_AT_12_VOLTS_MPS,
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
