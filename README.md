# Quasar
Robot code for 9105 TechnoTalons 2024 robot

This project is designed around the usage of [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit?tab=readme-ov-file). AdvantageKit requires all "inputs" (sensor readings, joystick movements, ete.) to be recorded to a log file. To do this efficiently, inputs are separated into different "IO" implementations. Below is the src file structure of the project:
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
More files will be added as needed.

As you can tell from the structure of the robot project, there are five subsystems.

## Drive
The drive subsystem is a 4-L3-MK4i module based swerve. The chassis is **27"x27"**. It uses two NEO Brushless Motors for each module to control the angle position and drive velocity of the swerve module. Each motor is controlled by a CAN SparkMax motor controller.

The command to drive the swerve generates chassis speeds from a field-relative orientation (front is facing away from you alliance wall). The command takes in joystick inputs to calculate a trajectory and feeds the desired **speeds** into the drive subsystem.