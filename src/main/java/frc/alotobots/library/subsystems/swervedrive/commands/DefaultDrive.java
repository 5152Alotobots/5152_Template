//   ____  _ ____ ____       _    _        ___ _____ ___  ____     ___ _____ ____
//  | ___|/ | ___|___ \    / \  | |     / _ \_   _/ _ \| __) / _ \_   _/ ___|
//  |___ \| |___ \ __) | / _ \ | |    | | | || || | | |  _ \|| | || | \___ \
//   ___) | |___) / __/   / ___ \| |__  | |_| || || |_| | |_) | |_| || |  ___) |
//  |____/|_|____/_____| /_/   \_\_____\___/ |_| \___/|____/ \___/ |_| |____/
//
//
// 2025 ALOTOBOTS FRC 5152
// Robot Code
package frc.alotobots.library.subsystems.swervedrive.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.OI;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;

public class DefaultDrive extends Command {
  private final SwerveDriveSubsystem swerveDriveSubsystem;

  public DefaultDrive(SwerveDriveSubsystem swerveDriveSubsystem) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    addRequirements(swerveDriveSubsystem);
  }

  @Override
  public void execute() {
    // Get inputs from OI (already field-relative)
    Translation2d linearVelocity = OI.getDriverLinearVelocity(swerveDriveSubsystem);
    double omega = OI.getDriverRotation();

    // Convert to chassis speeds
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * swerveDriveSubsystem.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * swerveDriveSubsystem.getMaxLinearSpeedMetersPerSec(),
            omega * swerveDriveSubsystem.getMaxAngularSpeedRadPerSec());

    // Apply speeds to drive
    swerveDriveSubsystem.runVelocity(speeds);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
