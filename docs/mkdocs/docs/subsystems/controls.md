# Controls Subsystem

The controls subsystem handles driver input processing and mapping. It provides sophisticated input processing for precise robot control.

## Input Processing Features

### Deadband Processing
The `JoystickUtilities` class provides several input processing methods:

- `joyDeadBnd`: Applies deadband with smooth transition
- `joySqrd`: Square input for finer control
- `joyScaled`: Scale input by a factor

## Configuration

### Deadband Constants
Located in `HMIDeadbands`:
- Forward/Backward: 0.1
- Strafe: 0.1

## Usage Examples

```java
// Apply deadband to raw joystick input
double processed = JoystickUtilities.joyDeadBnd(rawInput, HMIDeadbands.DRIVER_FWD_AXIS_DEADBAND);

// Square input for more precise control
double squared = JoystickUtilities.joySqrd(processed);

// Scale input
double scaled = JoystickUtilities.joyScaled(processed, 0.5);
```

## Best Practices

1. Always use deadband processing for joystick inputs
2. Consider using squared inputs for fine control
3. Test control sensitivity with different drivers
