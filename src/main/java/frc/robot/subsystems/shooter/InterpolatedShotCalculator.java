package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class InterpolatedShotCalculator {
    private InterpolatingDoubleTreeMap shotMap = new InterpolatingDoubleTreeMap();

    public InterpolatedShotCalculator() {
        shotMap.put(0.46, 0.0);
        shotMap.put(0.56, 4.0);
        shotMap.put(0.66, 5.0);
        shotMap.put(0.76, 6.0);
        shotMap.put(0.86, 7.0);
        shotMap.put(0.96, 8.0);
        shotMap.put(1.06, 9.0);
        shotMap.put(1.16, 10.0);
        shotMap.put(1.26, 11.0);
        shotMap.put(1.36, 12.0);
        shotMap.put(1.46, 13.0);
        shotMap.put(1.56, 14.0);
        shotMap.put(1.66, 15.0);
        shotMap.put(1.76, 16.0);
        shotMap.put(1.86, 17.0);
        shotMap.put(1.96, 18.0);
        shotMap.put(2.06, 19.0);
        shotMap.put(2.16, 20.0);
        shotMap.put(2.26, 21.0);
        shotMap.put(2.36, 22.0);
        shotMap.put(2.46, 23.0);
        shotMap.put(2.56, 24.0);
        shotMap.put(2.66, 25.0);
        shotMap.put(2.76, 26.0);
        shotMap.put(2.86, 27.0);
        shotMap.put(2.96, 28.0);
        shotMap.put(3.06, 29.0);
        shotMap.put(3.16, 30.0);
        shotMap.put(3.26, 31.0);
        shotMap.put(3.36, 32.0);
        shotMap.put(3.46, 33.0);
        shotMap.put(3.56, 34.0);
        shotMap.put(3.66, 35.0);
        shotMap.put(3.76, 36.0);
        shotMap.put(3.86, 37.0);
        shotMap.put(3.96, 38.0);
        shotMap.put(4.06, 39.0);
        shotMap.put(4.16, 40.0);
        shotMap.put(4.26, 41.0);
        shotMap.put(4.36, 42.0);
        shotMap.put(4.46, 43.0);
        shotMap.put(4.56, 44.0);
    }

    public Rotation2d getShotAngle(double distance) {
        return Rotation2d.fromDegrees( shotMap.get(distance).doubleValue() );
    }
}
