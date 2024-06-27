package frc.alotobots;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.Constants.SubsystemConfig;
import frc.alotobots.library.drivetrains.swerve_ctre.SwerveDriveSubsystem;

/**
 * This class contains methods to set up and manage autonomous commands for the robot.
 */
public class Auto {

    private final SwerveDriveSubsystem drivetrainSubsystem;
    private final SendableChooser<Command> autoChooser;

    /**
     * Constructs an AutoCommands object with the necessary subsystems.
     *
     * @param drivetrainSubsystem The SwerveDriveSubsystem
     */
    public Auto(SwerveDriveSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.autoChooser = new SendableChooser<>();
        configureAutoChooser();
        addAutoChooserToShuffleboard();
    }

    /**
     * Configures the autonomous command chooser.
     */
    private void configureAutoChooser() {
        if (SubsystemConfig.SWERVE_DRIVE_SUBSYSTEM_ENABLED && drivetrainSubsystem != null) {
            autoChooser.setDefaultOption("Default Auto", drivetrainSubsystem.getAutoPath("Default"));
            autoChooser.addOption("Complex Auto", drivetrainSubsystem.getAutoPath("Complex"));
            // Add more auto options here
        } else {
            autoChooser.setDefaultOption("No Auto Available", null);
        }
    }

    /**
     * Adds the auto chooser to the Shuffleboard.
     */
    private void addAutoChooserToShuffleboard() {
        ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
        driveTab.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(0, 0);
    }

    /**
     * Gets the selected autonomous command.
     *
     * @return The selected autonomous command
     */
    public Command getSelectedAutoCommand() {
        return autoChooser.getSelected();
    }
}