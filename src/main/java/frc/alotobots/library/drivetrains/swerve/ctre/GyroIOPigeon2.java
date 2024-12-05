package frc.alotobots.library.drivetrains.swerve.ctre;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.alotobots.Constants;
import frc.alotobots.library.drivetrains.swerve.ctre.mk4il22023.TunerConstants;
import java.util.Queue;

/** IO implementation for Pigeon2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yaw;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity;

  public GyroIOPigeon2() {
    pigeon = new Pigeon2(Constants.Robot.CanId.PIGEON_2_ID);
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.setYaw(0.0);
    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();
    yaw.setUpdateFrequency(TunerConstants.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(yaw, yawVelocity);
    inputs.connected = status.equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    // Log raw gyro data
    Logger.getInstance().processInputs("Drive/Gyro",
        Map.of(
            "connected", inputs.connected,
            "yawDegrees", yaw.getValueAsDouble(),
            "yawVelocityDegPerSec", yawVelocity.getValueAsDouble(),
            "statusCode", status.toString()
        ));

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
