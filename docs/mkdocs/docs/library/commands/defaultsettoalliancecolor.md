# DefaultSetToAllianceColor Command

## Purpose
DefaultSetToAllianceColor serves as the default command for the Bling subsystem, maintaining the robot's LED indicators at the current alliance color. This command runs continuously, including when the robot is disabled, ensuring consistent visual indication of alliance membership throughout a match.

## Required Subsystems
- [Bling Subsystem](../subsystems/bling.md)

## Constructor
```java
public DefaultSetToAllianceColor(BlingSubsystem subSysBling)
```

## Operation Details
The command operates with minimal complexity, setting the LED state during initialization and then maintaining that state. The LED updates are handled through the subsystem's periodic method, requiring no additional execution logic in the command itself.

The command is designed to run indefinitely and continue operation even when the robot is disabled. This behavior ensures the alliance color remains visible during all robot states, including pre-match setup and post-match periods.

## Special Characteristics
The command implements several important behaviors:
- Runs when disabled through the `runsWhenDisabled()` override
- Never completes execution (`isFinished()` returns false)
- Requires minimal resources as primary updates occur in the subsystem
- Maintains LED state without continuous commanding

## Usage Example
```java
BlingSubsystem bling = new BlingSubsystem();
bling.setDefaultCommand(new DefaultSetToAllianceColor(bling));
```

This command is typically set as the default command for the Bling subsystem during robot initialization, ensuring the alliance color display operates as the baseline behavior when no other LED commands are running.
