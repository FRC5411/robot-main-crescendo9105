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
        {Units.inchesToMeters(110.0), 40.3},
        {Units.inchesToMeters(120.0), 39.75},
        {Units.inchesToMeters(130.0), 38.5}
    };

    public static double[][] redShotMap = blueShotMap;
}
