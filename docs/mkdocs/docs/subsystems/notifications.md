# Elastic Notification System

The Elastic notification system provides real-time debugging and status notifications during robot operation.

## Features

- Multiple notification levels
- Customizable display duration
- Configurable notification size
- JSON-based message formatting

## Usage Example

```java
// Create and send a basic notification
new Elastic.ElasticNotification()
    .withLevel(NotificationLevel.INFO)
    .withTitle("System Status")
    .withDescription("Drivetrain initialized")
    .withDisplayMilliseconds(3000)
    .send();
```

## Notification Properties

- Level: Severity of the notification
- Title: Brief header text
- Description: Detailed message
- Display Time: Duration in milliseconds
- Dimensions: Width and height of notification

## Best Practices

1. Use appropriate notification levels (INFO, WARNING, ERROR)
2. Keep messages concise and informative
3. Set appropriate display durations
4. Group related notifications logically
