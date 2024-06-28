package frc.alotobots.library.drivetrains.swerve.ctre;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.alotobots.Constants;

import java.util.Map;

public class SwerveDriveTelemetry {
    private final ShuffleboardTab driveTab;
    private final ShuffleboardLayout poseList;
    private final ShuffleboardLayout tunableParamsList;
    private final Field2d field;

    // Tunable parameters
    private SimpleWidget maxSpeedWidget;
    private SimpleWidget maxAngularSpeedWidget;
    private SimpleWidget driveKPWidget;
    private SimpleWidget driveKIWidget;
    private SimpleWidget driveKDWidget;
    private SimpleWidget turnKPWidget;
    private SimpleWidget turnKIWidget;
    private SimpleWidget turnKDWidget;

    public SwerveDriveTelemetry(SwerveDriveSubsystem swerveDrive) {
        this.driveTab = Shuffleboard.getTab("Drive");
        this.field = new Field2d();
        this.poseList = initializeShuffleboard(swerveDrive);
        this.tunableParamsList = initializeTunableParameters(swerveDrive);
    }

    private ShuffleboardLayout initializeShuffleboard(SwerveDriveSubsystem swerveDrive) {
        ShuffleboardLayout poseList = driveTab
                .getLayout("Pose", BuiltInLayouts.kList)
                .withSize(2, 2)
                .withPosition(0, 4)
                .withProperties(Map.of("Label position", "LEFT"));

        poseList.addNumber("Pose X", swerveDrive.getPose()::getX);
        poseList.addNumber("Pose Y", swerveDrive.getPose()::getY);
        poseList.addNumber("Rotation", swerveDrive::getYaw);
        driveTab.add("Field", field)
                .withPosition(2, 0)
                .withSize(12, 5);

        driveTab.addBoolean("Tuning Mode", () -> Constants.Robot.TUNE_MODE)
                .withPosition(4, 5)
                .withSize(2, 1);
        driveTab.addBoolean("Flip Path", swerveDrive::getFlipPath)
                .withPosition(6, 5)
                .withSize(2, 1);

        return poseList;
    }

    private ShuffleboardLayout initializeTunableParameters(SwerveDriveSubsystem swerveDrive) {
        ShuffleboardLayout tunableParamsList = driveTab
                .getLayout("Tunable Parameters", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0)
                .withProperties(Map.of("Label position", "LEFT"));

        maxSpeedWidget = tunableParamsList.add("Max Speed (m/s)", swerveDrive.getMaxSpeed());
        maxAngularSpeedWidget = tunableParamsList.add("Max Angular Speed (rad/s)", swerveDrive.getMaxAngularSpeed());
        driveKPWidget = tunableParamsList.add("Drive kP", swerveDrive.getDriveKP());
        driveKIWidget = tunableParamsList.add("Drive kI", swerveDrive.getDriveKI());
        driveKDWidget = tunableParamsList.add("Drive kD", swerveDrive.getDriveKD());
        turnKPWidget = tunableParamsList.add("Turn kP", swerveDrive.getTurnKP());
        turnKIWidget = tunableParamsList.add("Turn kI", swerveDrive.getTurnKI());
        turnKDWidget = tunableParamsList.add("Turn kD", swerveDrive.getTurnKD());

        return tunableParamsList;
    }

    public void updateShuffleboard(SwerveDriveSubsystem swerveDrive) {
        field.setRobotPose(swerveDrive.getPose());

        if (Constants.Robot.TUNE_MODE) {
            updateTunableParameters(swerveDrive);
        }
    }

    private void updateTunableParameters(SwerveDriveSubsystem swerveDrive) {
        double newMaxSpeed = maxSpeedWidget.getEntry().getDouble(swerveDrive.getMaxSpeed());
        double newMaxAngularSpeed = maxAngularSpeedWidget.getEntry().getDouble(swerveDrive.getMaxAngularSpeed());

        if (newMaxSpeed != swerveDrive.getMaxSpeed() || newMaxAngularSpeed != swerveDrive.getMaxAngularSpeed()) {
            System.out.println("Hot reload of SwerveMaxSpeed");
            swerveDrive.setMaxSpeeds(newMaxSpeed, newMaxAngularSpeed);
        }

        double newDriveKP = driveKPWidget.getEntry().getDouble(swerveDrive.getDriveKP());
        double newDriveKI = driveKIWidget.getEntry().getDouble(swerveDrive.getDriveKI());
        double newDriveKD = driveKDWidget.getEntry().getDouble(swerveDrive.getDriveKD());

        if (newDriveKP != swerveDrive.getDriveKP() || newDriveKI != swerveDrive.getDriveKI() || newDriveKD != swerveDrive.getDriveKD()) {
            System.out.println("Hot reload of SwerveDrivePID");
            swerveDrive.setDrivePID(newDriveKP, newDriveKI, newDriveKD);
        }

        double newTurnKP = turnKPWidget.getEntry().getDouble(swerveDrive.getTurnKP());
        double newTurnKI = turnKIWidget.getEntry().getDouble(swerveDrive.getTurnKI());
        double newTurnKD = turnKDWidget.getEntry().getDouble(swerveDrive.getTurnKD());

        if (newTurnKP != swerveDrive.getTurnKP() || newTurnKI != swerveDrive.getTurnKI() || newTurnKD != swerveDrive.getTurnKD()) {
            System.out.println("Hot reload of SwerveTurnPID");
            swerveDrive.setTurnPID(newTurnKP, newTurnKI, newTurnKD);
        }
    }
}
