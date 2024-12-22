# Robot Quickstart Guide

This guide provides step-by-step instructions for configuring and tuning your robot's swerve drive system.

## 1. Swerve Drive Configuration

### 1.1 CAN ID Management

> **Important**: Proper CAN ID management is critical for reliable robot operation
Before beginning any swerve configuration, verify all CAN IDs:

1. Check current CAN IDs in `Constants.java` under `CanId`:
   ```java
   public static class CanId {
       public static final int FRONT_LEFT_DRIVE_MTR_CAN_ID = 1;
       public static final int FRONT_LEFT_STEER_MTR_CAN_ID = 2;
       public static final int FRONT_LEFT_STEER_CAN_CODER_CAN_ID = 3;
       // ... etc
   }
   ```

2. If your physical devices have different IDs:
    - DO NOT change the IDs in code
    - Use Phoenix Tuner X to update the device IDs to match the code
    - This maintains backwards compatibility with previous configurations
    - Helps prevent confusion when sharing code or reverting changes

3. Best Practices:
    - Keep a CAN ID spreadsheet/document for your team
    - Label physical devices with their CAN IDs
    - Use Phoenix Tuner X's network view to verify all devices are visible
    - Test communication with each device before proceeding

4. Common Issues:
    - If devices aren't showing up, check CAN termination resistors
    - Verify firmware versions are up to date
    - Ensure power and CAN wire connections are secure

### 1.2 New Season Setup
When starting a new season:
1. Copy the previous year's tuner constants file (e.g., `TunerConstants2023.java`)
2. Rename it to match the new year (e.g., `TunerConstants2024.java`)
3. Update the class name in the file to match
4. Update any imports referencing the old class name
5. Update the robot's main Constants file:
   ```java
   // In Constants.java
   public static final TunerConstants tunerConstants = new TunerConstants2024();
   // or if using different configurations for simulation:
   public static final TunerConstants tunerConstants = 
       Constants.currentMode == Mode.SIM ? 
           new TunerConstantsSim() : 
           new TunerConstants2024();
   ```
   This static field is used throughout the codebase to access swerve configurations.

6. Keep both files - old constants serve as a reference and backup

### 1.3 Module Configuration
The swerve drive configuration is generated using Phoenix Tuner X's Swerve Generator tool:

1. Launch Phoenix Tuner X and connect to the robot
2. Open the Swerve Generator tool
3. Configure module hardware in Phoenix Tuner X:
    - Select your motor type:
        - Kraken X60
        - Falcon 500
    - Set gear ratios for drive and steer (based on your swerve module type)
    - Input wheel radius
    - Select encoder type:
        - CANcoder
        - CANcoder 2.0

4. Set CAN IDs for each module:
    - Drive Motor
    - Turn Motor
    - CANcoder
    - Ensure IDs match physical hardware

5. Zero all CANcoders:
    - Point all wheels toward the center of the robot:
        - Bevel gears (black side) should face inward on all modules
        - If available, use the module's zeroing holes with a 3/16" bar
        - If no zeroing holes, align wheels visually toward robot center
    - Double check all wheels are properly aligned
    - Use Tuner X to record offsets
    - Verify offsets are correct by powering off/on and checking wheel alignment

6. Copying Generated Constants:
    - The generator creates a complete constants file with all swerve configurations
    - In your project, locate the `GeneratedConstants` inner class within your new `TunerConstants2024.java`
    - **DO NOT** replace the entire `TunerConstants2024.java` file
    - Only copy the constant values from the generator output into your existing `GeneratedConstants` class
    - This includes:
        - Module offsets
        - Gear ratios
        - PID gains
        - Basic configurations
        - Motor and encoder IDs
    - **IMPORTANT**: Delete or do not copy the `createSwerveDrive()` method
        - This method is not used in our implementation
        - We handle swerve drive creation differently
    - Optional: Replace hardcoded CAN IDs with Constants references
      ```java
      // Replace generated values like:
      public static final int kFrontLeftDriveMotorId = 1;
      
      // With references to Constants:
      public static final int kFrontLeftDriveMotorId = Constants.CanId.FRONT_LEFT_DRIVE_MTR_CAN_ID;
      ```
      This makes ID management more centralized and maintainable
    - Leave all custom configurations in `CustomConstants` unchanged
        - These values (like translation PID, rotation PID, constraints) will be tuned later
        - Do not overwrite them with generated values
    - The `TunerConstants2024.java` file implements `TunerConstants.java` and should maintain its structure

### 1.4 Drive System Testing
After configuring modules and before PID tuning, verify basic functionality:

1. Initial Movement Check:
    - Deploy code with default PID values:
      ```java
      // Default steer (turn) gains
      private static final Slot0Configs steerGains = new Slot0Configs()
          .withKP(100)
          .withKI(0)
          .withKD(0.5)
          .withKS(0.2)
          .withKV(1.59)
          .withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
 
      // Default drive gains
      private static final Slot0Configs driveGains = new Slot0Configs()
          .withKP(0.1)
          .withKI(0)
          .withKD(0)
          .withKS(0)
          .withKV(0.124);
      ```
    - Lower robot onto ground
    - Very slowly test each movement:
        - Forward/Backward
        - Left/Right strafe
        - Rotation
        - **Note**: Movement may not be perfect yet as we use 254's Swerve Trajectory Setpoint Generator, which requires tuned PathPlanner constraints (we'll configure these in a later step)
    - Watch for:
        - All wheels spinning correct directions
        - No grinding or unusual noises
        - Basic responsiveness to commands

2. Module Behavior Verification:
    - Watch each module individually
    - Verify wheels point in expected directions
    - Make sure drive motors move in sync
    - Check for any modules "fighting" each other

### 1.5 Drive Characterization
Now we'll determine the feed-forward values:

1. Setup:
    - Place robot in an open space
    - Connect to robot via Driver Station
    - Open AdvantageScope

2. Run Characterization:
   Select the "Drive Simple FF Characterization" auto routine.
   Enable autonomous
   Disable after 10-15s
    - Command will gradually increase voltage
    - Watch modules for smooth acceleration
    - Let run until reaching full speed
    - Values will print to Driver Station console

3. Record Values:
    - Look for kS and kV output in console
    - Record both values for next step
    - Run 2-3 times to verify consistency

### 1.6 Drive Motor Tuning
Update PID and FF values based on characterization:

1. Initial Feed Forward:
   ```java
   // In GeneratedConstants:
   public static final Slot0Configs driveGains = new Slot0Configs()
       .withKS(/* value from characterization */)
       .withKV(/* value from characterization */)
       .withKA(0.01);  // Start small
   ```

2. Base PID Values:
   ```java
   public static final Slot0Configs driveGains = driveGains
       .withKP(0.05)  // Start conservative
       .withKI(0.0)   // Usually not needed
       .withKD(0.0);  // Add only if needed
   ```

3. Testing Process:
    - Deploy code
    - Start with very slow movements
    - Test forward/backward/strafe
    - Watch for:
        - Smooth acceleration
        - No oscillation
        - Wheels staying in sync

4. PID Tuning Steps:
   a. If response is sluggish:
    - Increase kP by 0.02
    - Test movement
    - Repeat until responsive

   b. If modules oscillate:
    - Reduce kP by 25%
    - Add small kD (start 0.001)
    - Test movement

   c. If wheels fight each other:
    - Verify module offsets
    - Check wheel direction conventions
    - Reduce kP slightly

5. Velocity Tuning:
    - Test at 25% speed:
        - Adjust kP until responsive
        - Add kD if oscillating
    - Test at 50% speed:
        - Verify stability
        - Adjust if needed
    - Test at full speed:
        - Final verification
        - Fine-tune as needed

6. Final Testing:
    - Run full-speed maneuvers
    - Check quick direction changes
    - Verify smooth acceleration
    - Test rotation stability
    - Look for any wheel slip

Your final drive gains might look like:
```java
public static final Slot0Configs driveGains = new Slot0Configs()
    .withKP(0.09)
    .withKI(0.0)
    .withKD(0.001)
    .withKS(0.19437)
    .withKV(0.75843)
    .withKA(0.01);
```

### 1.7 Measuring Maximum Speed
After tuning the drive motors, you'll need to measure the robot's actual maximum speed:

1. Safety First:
    - Place the robot securely on blocks
    - Ensure all wheels are free to spin
    - Clear the area around the robot
    - Have emergency stop ready

2. Using Phoenix Tuner X:
    - Connect to robot
    - Open "Plot" view
    - Select drive motors
    - Use "Control" tab
    - Set to "Voltage Control"
    - Command 12V to drive motors

3. Measuring Speed:
    - Let motors reach full speed
    - Record velocity from plot
    - Take measurements from all modules
    - Average the results
    - Convert to your preferred units (meters/second)

4. Updating Constants:
    - Open your TunerConstants file
    - Update the speed value:
    ```java
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.02);
    ```
    - This value is used for:
        - Path planning
        - Autonomous routines
        - Speed limiting
        - Controller scaling

5. Verification:
    - Deploy updated code
    - Test at various speed percentages
    - Verify autonomous paths work correctly
    - Check that speed limits are respected

### 1.8 Slip Current Measurement
After setting maximum speed, we need to determine the current limit that prevents wheel slip:

1. Setup:
    - Position robot against a solid wall
    - Open AdvantageScope
    - Create plots for:
        - Drive motor current (/Drive/Module.../DriveCurrentAmps)
        - Drive velocity (/Drive/Module.../DriveVelocityRadPerSec)

2. Measurement Process:
    - Gradually increase forward throttle
    - Watch velocity plot carefully
    - When velocity suddenly increases (wheel slip), note the current
    - This is your slip current threshold

3. Updating Constants:
    - Open your TunerConstants file
    - Set kSlipCurrent to the measured value:
    ```java
    public static final double kSlipCurrent = 40.0; // Amperes - adjust to your measured value
    ```
    - This prevents wheel slip during high-torque maneuvers

4. Verification:
    - Deploy updated code
    - Test aggressive movements
    - Verify wheels maintain traction
    - Check performance on different surfaces

### 1.9 Robot Mass Configuration
After setting up slip current limits, configure the robot's mass for PathPlanner:

1. Measure Robot Mass:
    - Weigh the complete robot
    - Include all components:
        - Battery
        - Bumpers
        - Game pieces (if carrying during auto)
    - Convert to kilograms

2. Update Configuration:
    - In your TunerConstants file, update the PathPlanner config:
    ```java
    public static final double ROBOT_MASS_KG = 34; // Update with your measured mass
    ```

### 1.10 Robot Moment of Inertia Measurement
After configuring robot mass, measure the robot's rotational inertia (MOI) for better path following:

1. Code Preparation:
    - Locate Module.java in your project
    - Find the runCharacterization method (around line 89)
    - Replace the existing method with:
    ```java
    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(new Rotation2d(constants.LocationX, constants.LocationY).plus(Rotation2d.kCCW_Pi_2));
    }
    ```
    - Deploy the updated code to the robot

2. Setup:
    - Clear a large, flat area
    - Place robot on smooth surface
    - Connect to robot via Driver Station
    - Open AdvantageScope for data logging

3. Data Collection:
    - Through dashboard autos, run each SysId routine:
        - Drive SysId (Quasistatic Forward)
        - Drive SysId (Quasistatic Reverse)
        - Drive SysId (Dynamic Forward)
        - Drive SysId (Dynamic Reverse)
    - Let each routine complete fully
    - Save logs after each run

4. Data Export:
    - Remove USB drive from robot
    - Connect to computer
    - Open AdvantageScope (v3.0.2 or later)
    - Load the log file
    - Go to "File" > "Export Data..."
    - Configure export settings:
        - Format: "WPILOG"
        - Timestamps: "AdvantageKit Cycles"
        - Select necessary fields for SysId
    - Save the converted log file

5. SysId Analysis:
    - Open VSCode
    - Launch SysId tool:
        - Command Palette (Ctrl+Shift+P)
        - Search "WPILib: Start Tool"
        - Select "SysId"
    - Load the exported log file
    - Record these values:
        - kA_angular (V/(rad/s²))
        - kA_linear (V/(m/s²))

6. Calculate MOI:
    Use this formula:
    ```
    MOI = mass * (trackwidth/2) * (kA_angular/kA_linear)
    ```
    Where:
    - mass = robot mass in kg
    - trackwidth = largest distance between wheel centers
    - kA_angular = angular acceleration feedforward
    - kA_linear = linear acceleration feedforward

7. Update Configuration:
    - In your TunerConstants file, update the MOI constant:
    ```java
    public static final double ROBOT_MOI_KGM2 = 4.24; // Update with calculated value
    ```
    This value represents the robot's resistance to rotational acceleration

8. Verification:
    - Deploy updated code
    - Test rotational movements
    - Verify smooth acceleration in turns
    - Check path following accuracy with rotation
    - Pay special attention to:
        - Turn initiation smoothness
        - Stopping accuracy
        - Multi-axis movement stability

### 1.11 Wheel Coefficient of Friction Measurement
After configuring mass and MOI, measure the wheel coefficient of friction for accurate path following:

1. Setup:
    - Clear a flat, consistent surface area
    - Place robot on competition surface material
    - Open AdvantageScope for data logging
    - Create plots for:
        - Module velocities
        - Drive motor currents
        - Robot acceleration

2. Measurement Process:
    - Run the "Wheel COF Characterization" autonomous routine
    - Command will:
        - Gradually increase drive power
        - Monitor wheel slip
        - Calculate static and dynamic COF
    - Test will run for about 10 seconds
    - Results print to Driver Station console

3. Update Configuration:
    - In your TunerConstants file, update the COF constant:
    ```java
    public static final double WHEEL_COF = 1.1;  // Update with measured value
    ```
    This value helps PathPlanner calculate maximum achievable accelerations

4. Verification:
    - Deploy updated code
    - Test aggressive path following
    - Verify wheels maintain traction
    - Check acceleration limits are respected

### 1.12 Turn Motor Tuning
The turn (steering) motors require different tuning approaches than drive motors:

1. Initial Setup:
    - Ensure wheels can rotate freely
    - Start with conservative values:
    ```java
    public static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(40)
        .withKI(0)
        .withKD(0.1)
        .withKS(0.12)
        .withKV(0.102);
    ```

2. Position Control Testing:
    - Deploy code with initial values
    - Use driver station to command 90-degree turns
    - Watch for:
        - Quick response without overshooting
        - No oscillation at target position
        - Smooth acceleration and deceleration
        - Ability to hold position when pushed

3. PID Tuning Process:
   a. Start with Position Control:
    - Increase kP until wheels respond quickly
    - If oscillating, reduce kP by 25%
    - Add kD to dampen oscillations (start with kD = kP * 0.01)
    - Adjust until wheels snap to position without overshooting

   b. Add Feed Forward:
    - Start with small kS (0.1-0.2)
    - Increase if modules struggle to overcome friction
    - Add kV based on max velocity requirements
    - Keep kA at 0 unless needed for high acceleration

4. Common Issues:
    - Oscillation: Reduce kP or increase kD
    - Slow response: Increase kP
    - Position drift: Increase kS slightly
    - Overshooting: Increase kD or reduce kP
    - Grinding noise: Check mechanical alignment and reduce gains

5. Final Verification:
    - Test rapid direction changes
    - Verify holding position under load
    - Check for smooth motion at various speeds
    - Ensure all modules perform consistently

Your final turn gains might look like:
```java
public static final Slot0Configs steerGains = new Slot0Configs()
    .withKP(55)
    .withKI(0)
    .withKD(0.2)
    .withKS(0.12)
    .withKV(0.102)
    .withKA(0)
    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
```

