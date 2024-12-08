package frc.alotobots.library.subsystems.swervedrive.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.OI;
import frc.alotobots.library.subsystems.swervedrive.Drive;

public class DefaultDrive extends Command {
    private final Drive drive;

    public DefaultDrive(Drive drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        // Get inputs from OI (already field-relative)
        Translation2d linearVelocity = OI.getDriverLinearVelocity(drive);
        double omega = OI.getDriverRotation();

        // Convert to chassis speeds
        ChassisSpeeds speeds = new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega * drive.getMaxAngularSpeedRadPerSec()
        );

        // Apply speeds to drive
        drive.runVelocity(speeds);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
