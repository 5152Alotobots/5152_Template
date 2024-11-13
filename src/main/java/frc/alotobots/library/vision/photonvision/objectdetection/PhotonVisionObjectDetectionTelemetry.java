package frc.alotobots.library.vision.photonvision.objectdetection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PhotonVisionObjectDetectionTelemetry {
    private static final String SUBSYSTEM_KEY = "PhotonVision ObjectDetection/";

    public void update(PhotonVisionObjectDetectionSubsystem subsystem) {
        SmartDashboard.putBoolean(SUBSYSTEM_KEY + "Has Targets", subsystem.hasTargets());
        
        if (subsystem.hasTargets()) {
            var targets = subsystem.getLatestTargets();
            SmartDashboard.putNumber(SUBSYSTEM_KEY + "Target Count", targets.size());
            
            if (!targets.isEmpty()) {
                var bestTarget = targets.get(0);
                SmartDashboard.putNumber(SUBSYSTEM_KEY + "Best Target/Yaw", bestTarget.getYaw());
                SmartDashboard.putNumber(SUBSYSTEM_KEY + "Best Target/Pitch", bestTarget.getPitch());
                SmartDashboard.putNumber(SUBSYSTEM_KEY + "Best Target/Area", bestTarget.getArea());
                SmartDashboard.putNumber(SUBSYSTEM_KEY + "Best Target/Skew", bestTarget.getSkew());
            }
        }
    }
}
