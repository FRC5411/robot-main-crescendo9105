# Project Structure
This code is organized as a [WPILib](https://docs.wpilib.org/en/stable/index.html) [Command-based](https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html) project. This program makes use of [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit?tab=readme-ov-file) to log and replay data.

## Telemetry & data
This year, our team wanted to use AdvantageKit for it's ability to [log](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/RECORDING-INPUTS.md) and replay match data. It's ease of use made it an obvious solution for the dilema of inaccurate or inconsistent logged data. AdvantageKit is able to **accurately** log and replay data because all *inputs* (sensor data, joystick movements, ete.) are recorded using deterministic timestamps. Then, those inputs can be replayed and compared against code to generate the same set of *outputs* (motor voltages, triggering commands, ete.).

In order to accurately log these inputs, they must be put into an IO layer. While this does result in more files, the benefits of accurate telemetry and data are well worth it. For more information see [AdvantageKit docs](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/WHAT-IS-ADVANTAGEKIT.md).

## File tree
In order to better organize this project, a filetree was developed:

```
.
└── main/
    ├── deploy/
    │   └── pathplanner/
    │       ├── autos/
    │       │   └── Calibration.auto
    │       └── paths/
    │           ├── Striaght.path
    │           └── Curve.path
    └── java/
        └── frc/
            └── robot/
                ├── commands/
                │   ├── ClimbCommands.java
                │   ├── IntakeCommands.java
                │   ├── PathCommands.java
                │   └── SwerveCommands.java
                └── subsystems/
                    ├── climb/
                    │   ├── Climb.java
                    │   ├── ClimbIO.java
                    │   ├── ClimbIOSim.java
                    │   ├── ClimbIOSparkMax.java
                    │   └── ClimbVisualizer.java
                    ├── drive/
                    │   ├── Drive.java
                    │   ├── GyroIO.java
                    │   ├── GyroIOPigeon2.java
                    │   ├── Module.java
                    │   ├── ModuleIO.java
                    │   ├── ModuleIOSim.java
                    │   ├── ModuleIOSparkMax.java
                    │   ├── PhoenixOdometryThread.java
                    │   └── SparkMaxOdometryThread.java
                    ├── intake/
                    │   ├── Intake.java
                    │   ├── IntakeIO.java
                    │   ├── IntakeIOSim.java
                    │   └── IntakeIOSparkMax.java
                    ├── shooter/
                    │   ├── angler/
                    │   │   ├── AnglerKinematics.java
                    │   │   ├── AnglerIO.java
                    │   │   ├── AnglerIOSim.java
                    │   │   └── AnglerIOSparkMax.java
                    │   ├── indexer/
                    │   │   ├── IndexerIO.java
                    │   │   ├── IndexerIOSim.java
                    │   │   └── IndexerIOSparkMax.java
                    │   ├── launcher/
                    │   │   ├── LauncherIO.java
                    │   │   ├── LauncherIOSim.java
                    │   │   ├── LauncherIOFalcon500.java
                    │   │   └── TrajectorySolver.java
                    │   └── Shooter.java
                    ├── vision/
                    │   ├── Vision.java
                    │   ├── VisionIO.java
                    │   └── VisionIOLimelight.java
                    ├── Constants.java
                    ├── Main.java
                    ├── Robot.java
                    └── RobotContainer.java
```

More files will be added, removed, or updated as needed.
