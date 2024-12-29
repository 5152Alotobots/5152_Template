/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2024 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.alotobots.library.subsystems.vision.questnav.io;

import edu.wpi.first.networktables.*;

public class OculusIOReal implements OculusIO {
  private final NetworkTable nt4Table;
  private final IntegerSubscriber questMiso;
  private final IntegerPublisher questMosi;
  private final IntegerSubscriber questFrameCount;
  private final DoubleSubscriber questTimestamp;
  private final FloatArraySubscriber questPosition;
  private final FloatArraySubscriber questQuaternion;
  private final FloatArraySubscriber questEulerAngles;
  private final DoubleSubscriber questBatteryPercent;
  private final DoubleArrayPublisher resetPosePub;

  public OculusIOReal() {
    nt4Table = NetworkTableInstance.getDefault().getTable("questnav");
    questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
    questMosi = nt4Table.getIntegerTopic("mosi").publish();
    questFrameCount = nt4Table.getIntegerTopic("frameCount").subscribe(0);
    questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0);
    questPosition =
        nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
    questQuaternion =
        nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
    questEulerAngles =
        nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});
    questBatteryPercent = nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0);

    // Publishers for pose reset
    resetPosePub = nt4Table.getDoubleArrayTopic("resetpose").publish();
  }

  @Override
  public void updateInputs(OculusIOInputs inputs) {
    inputs.position = questPosition.get();
    inputs.quaternion = questQuaternion.get();
    inputs.eulerAngles = questEulerAngles.get();
    inputs.timestamp = questTimestamp.get();
    inputs.frameCount = (int) questFrameCount.get();
    inputs.batteryPercent = questBatteryPercent.get();
    inputs.misoValue = (int) questMiso.get();
  }

  @Override
  public void setMosi(int value) {
    questMosi.set(value);
  }

  @Override
  public void setResetPose(double x, double y, double rotation) {
    // First value of 1 indicates "ready"
    resetPosePub.set(new double[] {x, y, rotation});
  }
}
