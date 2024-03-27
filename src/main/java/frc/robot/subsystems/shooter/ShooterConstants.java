package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static double[][] blueShotMap = {
        {Units.inchesToMeters(37.0), 55.0},
        {Units.inchesToMeters(47.0), 53.0},
        {Units.inchesToMeters(57.0), 52.0},
        {Units.inchesToMeters(67.0), 49.5},
        {Units.inchesToMeters(77.0), 47.0},
        {Units.inchesToMeters(87.0), 44.5},
        {Units.inchesToMeters(97.0), 42.0},
        {Units.inchesToMeters(107.0), 40.5},
        {Units.inchesToMeters(109.5), 39.0},
        {Units.inchesToMeters(127.0), 37.5}, // Might be bad
        {Units.inchesToMeters(144.0), 36.0}, // Might be bad
        {Units.inchesToMeters(154.0), 0},
    };

    public static double[][] redShotMap = blueShotMap;
}
