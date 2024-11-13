package frc.alotobots.library.vision.photonvision.objectdetection;

import lombok.experimental.UtilityClass;

@UtilityClass
public class PhotonVisionObjectDetectionConstants {
    public static final String CAMERA_NAME = "OBJ_CAM";
    
    // Detection confidence threshold (0-1)
    public static final double MIN_CONFIDENCE = 0.5;
    
    // Maximum distance to track objects (meters)
    public static final double MAX_TRACK_DISTANCE = 6.0;
    
    // Pose ambiguity threshold
    public static final double MAX_POSE_AMBIGUITY = 0.2;
    
    // Camera mounting position relative to robot center (meters)
    public static final double CAMERA_HEIGHT_METERS = 0.5;
    public static final double CAMERA_PITCH_RADIANS = 0.0;
    public static final double CAMERA_ROLL_RADIANS = 0.0;
    public static final double CAMERA_YAW_RADIANS = 0.0;
    public static final double CAMERA_X_OFFSET = 0.0;
    public static final double CAMERA_Y_OFFSET = 0.0;
    public static final double CAMERA_Z_OFFSET = CAMERA_HEIGHT_METERS;
}
