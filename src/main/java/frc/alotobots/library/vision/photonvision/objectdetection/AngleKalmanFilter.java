package frc.alotobots.library.vision.photonvision.objectdetection;

/**
 * Implements a simple Kalman filter for angle tracking.
 * State vector: [angle, angular_velocity]
 */
public class AngleKalmanFilter {
    // State estimate
    private double angle;
    private double angularVelocity;
    
    // Estimate uncertainty
    private double pAngle = 1.0;
    private double pVelocity = 1.0;
    
    // Process noise
    private static final double Q_ANGLE = 0.001;
    private static final double Q_VELOCITY = 0.1;
    
    // Measurement noise
    private static final double R_ANGLE = 0.1;
    
    // Time step
    private static final double DT = 0.02; // 20ms
    
    public AngleKalmanFilter() {
        reset();
    }
    
    public void reset() {
        angle = 0;
        angularVelocity = 0;
        pAngle = 1.0;
        pVelocity = 1.0;
    }
    
    public void predict() {
        // State prediction
        angle += angularVelocity * DT;
        
        // Uncertainty prediction
        double temp = pAngle;
        pAngle += DT * (DT * pVelocity - temp);
        pVelocity += Q_VELOCITY;
        pAngle += Q_ANGLE;
    }
    
    public void update(double measurement) {
        // Kalman gain calculation
        double K = pAngle / (pAngle + R_ANGLE);
        
        // State update
        double innovation = measurement - angle;
        // Wrap innovation to [-π, π]
        innovation = Math.atan2(Math.sin(innovation), Math.cos(innovation));
        
        angle += K * innovation;
        angularVelocity += (K * innovation) / DT;
        
        // Uncertainty update
        pAngle *= (1 - K);
        pVelocity *= (1 - K);
    }
    
    public double getAngle() {
        return angle;
    }
    
    public double getAngularVelocity() {
        return angularVelocity;
    }
}
