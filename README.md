# ED2025 Subsystems
## DriveTrain
All CTRE-Phoenix (v6)
- 8 Kraken (Cross The Road Electronics - CTRE) Motors
- 4 CTRE CANCoders
### Front Left Swerve Module (3 - numbers used by BAMF)
- Drive Motor CAN ID: 31
- Steer Motor CAN ID: 32
- Absolute CANCoder ID: 30

### Front Right Swerve Module (4)
- Drive Motor CAN ID: 21
- Steer Motor CAN ID: 22
- Absolute CANCoder ID: 20

### Back Left Swerve Module (2)
- Drive Motor CAN ID: 41
- Steer Motor CAN ID: 42
- Absolute CANCoder ID: 40

### Back Right Swerve Module (1)
- Drive Motor CAN ID: 11
- Steer Motor CAN ID: 12
- Absolute CANCoder ID: 10

## Lift / Elevator
All REVLib (No linear encoder)
- 2 NEO Motors
- top and bottom limit switches (safety)

## Grabber / Intake / THE Manipulator (combined subsystems?)
### Hand / Claw
All REVLib
- 2 NEO Motors
- Left Motor CAN ID: ??
- Right Motor CAN ID: ??
- coral contact limit switch
- algae contact limit switch

### Wrist
All REVLib (?? or is there a CANCoder or other type of abs encoder ??)
- Probably CANCoder
- 1 NEO Motor
- Motor CAN ID: ??
- 2 max ranges limit switches (safety)

## Climber
All REVLib
- 1 NEO Motors
- Left Motor CAN ID: ??
- Right Motor CAN ID: ??
- one stop limit switch (safety)

## Misc
- distance sensor (??)