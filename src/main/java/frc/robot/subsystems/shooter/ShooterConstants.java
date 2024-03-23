package frc.robot.subsystems.shooter;

public class ShooterConstants {
    public static double[][] blueShotMap = {
        {0.0, 55.0},
        {0.25, 52.0},
        {0.50, 48.0},
        {0.75, 45.0},
        {1.0, 42.5},
        {1.25, 40.0},
        {1.5, 38.5},
        {1.75, 37.0},
        {1.0, 35.6},
        {1.25, 34.5},
        {1.5, 33.5},
        {1.75, 32.0},
        {3.0, 31.1},
        {3.25, 30.3},
        {3.5, 29.9}
    };

    public static double[][] redShotMap = {
        {blueShotMap[0][0], blueShotMap[0][1]},
        {blueShotMap[1][0], blueShotMap[1][1]},
        {blueShotMap[2][0], blueShotMap[2][1]},
        {blueShotMap[3][0], blueShotMap[3][1]},
        {blueShotMap[4][0], blueShotMap[4][1]},
        {blueShotMap[5][0], blueShotMap[5][1]},
        {blueShotMap[6][0], blueShotMap[6][1]},
        {blueShotMap[7][0], blueShotMap[7][1]},
        {blueShotMap[8][0], blueShotMap[8][1]},
        {blueShotMap[9][0], blueShotMap[9][1]},
        {blueShotMap[10][0], blueShotMap[10][1]},
        {blueShotMap[11][0], blueShotMap[11][1]},
        {blueShotMap[12][0], blueShotMap[12][1]},
        {blueShotMap[13][0], blueShotMap[13][1]},
        {blueShotMap[14][0], blueShotMap[14][1]}
    };
}
