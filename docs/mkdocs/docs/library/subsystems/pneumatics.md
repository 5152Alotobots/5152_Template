# Pneumatics Subsystem

The pneumatics subsystem handles all pneumatic actuators on the robot. This includes:

## Features

- Solenoid control
- Pressure monitoring
- Compressor management

## Configuration

The pneumatics system is configured through the `PneumaticsSubsystemConstants` class.

## Best Practices

1. Always monitor system pressure
2. Use double solenoids when possible for positive control
3. Implement safety timeouts for actuations
