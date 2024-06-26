package frc.alotobots.library.vision.limelight;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.alotobots.library.drivetrains.swerve_ctre.SwerveDriveSubsystem;
import lombok.Getter;
import lombok.AllArgsConstructor;

/**
 * Represents an object detected by the Limelight vision system.
 */
@AllArgsConstructor
public class DetectedObject {
    private static SwerveDriveSubsystem drive;

    @Getter
    private final Pose3d pose;

    @Getter
    private final ObjectType type;

    /**
     * Enum representing the types of objects that can be detected.
     */
    public enum ObjectType {
        NOTE, RED_ROBOT, BLUE_ROBOT, NONE
    }

    /**
     * Sets the drivetrain to use for pose calculations.
     *
     * @param drive The SwerveDrive subsystem.
     */
    public static void setDrive(SwerveDriveSubsystem drive) {
        DetectedObject.drive = drive;
    }

    /**
     * Creates a new DetectedObject with default attributes.
     */
    public DetectedObject() {
        this(new Pose3d(), ObjectType.NONE);
    }

    /**
     * Creates a new DetectedObject based on camera offsets and robot-to-camera transform.
     *
     * @param xOffset The x offset from the camera to the object in radians.
     * @param yOffset The y offset from the camera to the object in radians.
     * @param type The type of the detected object.
     * @param robotToCamera The transformation from the robot to the camera.
     */
    public DetectedObject(double xOffset, double yOffset, ObjectType type, Transform3d robotToCamera) {
        this(calculateObjectPose(xOffset, yOffset, robotToCamera), type);
    }

    /**
     * Creates a new DetectedObject based on camera offsets and robot-to-camera transform.
     *
     * @param xOffset The x offset from the camera to the object in radians.
     * @param yOffset The y offset from the camera to the object in radians.
     * @param type The type of the detected object as a long value.
     * @param robotToCamera The transformation from the robot to the camera.
     */
    public DetectedObject(double xOffset, double yOffset, long type, Transform3d robotToCamera) {
        this(xOffset, yOffset, getType(type), robotToCamera);
    }

    /**
     * Calculates the pose of the detected object.
     *
     * @param xOffset The x offset from the camera to the object in radians.
     * @param yOffset The y offset from the camera to the object in radians.
     * @param robotToCamera The transformation from the robot to the camera.
     * @return The calculated Pose3d of the object.
     */
    private static Pose3d calculateObjectPose(double xOffset, double yOffset, Transform3d robotToCamera) {
        Translation3d translation = new Translation3d(1, new Rotation3d(0, yOffset, -xOffset));
        translation = translation.rotateBy(robotToCamera.getRotation());

        if (drive != null) {
            translation = applyDriveTransformation(translation);
        }

        return new Pose3d(translation, new Rotation3d());
    }

    /**
     * Applies the drive transformation to the object's translation.
     *
     * @param translation The initial translation of the object.
     * @return The updated translation after applying drive transformation.
     */
    private static Translation3d applyDriveTransformation(Translation3d translation) {
        translation = translation.rotateBy(new Rotation3d(0, 0, drive.getState().Pose.getRotation().getRadians()));
        Translation2d drivePose = drive.getState().Pose.getTranslation();
        return translation.plus(new Translation3d(drivePose.getX(), drivePose.getY(), 0));
    }

    /**
     * Converts a long value to an ObjectType.
     *
     * @param type The type as a long value.
     * @return The corresponding ObjectType.
     */
    public static ObjectType getType(long type) {
        return switch ((int) type) {
            case 0 -> ObjectType.NOTE;
            case 1 -> ObjectType.RED_ROBOT;
            case 2 -> ObjectType.BLUE_ROBOT;
            default -> ObjectType.NONE;
        };
    }

    /**
     * Checks if the detected object is a game piece.
     *
     * @return True if the object is a note, false otherwise.
     */
    public boolean isGamePiece() {
        return type == ObjectType.NOTE;
    }

    /**
     * Checks if the detected object is a robot.
     *
     * @return True if the object is a red or blue robot, false otherwise.
     */
    public boolean isRobot() {
        return type == ObjectType.RED_ROBOT || type == ObjectType.BLUE_ROBOT;
    }

    /**
     * Checks if the detected object is a robot on the same alliance.
     *
     * @return True if the object is a robot on the same alliance, false otherwise.
     */
    public boolean isSameAllianceRobot() {
        return DriverStation.getAlliance().map(alliance ->
                type == (alliance == Alliance.Red ? ObjectType.RED_ROBOT : ObjectType.BLUE_ROBOT)
        ).orElse(true);
    }

    /**
     * Checks if the detected object is a robot on the other alliance.
     *
     * @return True if the object is a robot on the other alliance, false otherwise.
     */
    public boolean isOtherAllianceRobot() {
        return DriverStation.getAlliance().map(alliance ->
                type == (alliance == Alliance.Red ? ObjectType.BLUE_ROBOT : ObjectType.RED_ROBOT)
        ).orElse(false);
    }

    /**
     * Gets the distance from the center of the robot to the object.
     *
     * @return The distance in meters, or 0 if the drive is not set.
     */
    public double getDistance() {
        return drive != null ?
                drive.getState().Pose.getTranslation().getDistance(pose.getTranslation().toTranslation2d()) : 0;
    }

    /**
     * Gets the field-relative angle from the robot to the object.
     *
     * @return The angle in radians, or 0 if the drive is not set.
     */
    public double getAngle() {
        if (drive != null) {
            return Math.atan2(
                    pose.getY() - drive.getState().Pose.getY(),
                    pose.getX() - drive.getState().Pose.getX()
            );
        }
        return 0;
    }

    /**
     * Gets the angle relative to the robot (0 is in front, positive counterclockwise).
     *
     * @return The relative angle in radians, or 0 if the drive is not set.
     */
    public double getRelativeAngle() {
        if (drive != null) {
            double angle = getAngle() - drive.getState().Pose.getRotation().getRadians();
            return Math.IEEEremainder(angle, 2 * Math.PI);
        }
        return 0;
    }

    @Override
    public String toString() {
        return String.format("%s at (%.2f, %.2f, %.2f)", type, pose.getX(), pose.getY(), pose.getZ());
    }
}