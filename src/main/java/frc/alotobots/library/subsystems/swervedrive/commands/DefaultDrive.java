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
        // Get linear velocity from OI
        Translation2d linearVelocity = OI.getDriverLinearVelocity();
        
        // Get rotation from OI
        double omega = OI.getDriverRotation();

        // Convert to field relative speeds
        ChassisSpeeds speeds = new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega * drive.getMaxAngularSpeedRadPerSec()
        );

        // Handle alliance-based field orientation
        boolean isFlipped = DriverStation.getAlliance().isPresent() 
            && DriverStation.getAlliance().get() == Alliance.Red;
        
        speeds.toRobotRelativeSpeeds(
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) 
                     : drive.getRotation()
        );

        // Apply speeds to drive
        drive.runVelocity(speeds);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
