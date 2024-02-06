package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.shooter.Angler.AnglerConstants;

public class ScrewArmKinematics {
  public static double getLengthAlongScrew(Rotation2d pivotAngle) {
    return AnglerConstants.kPivotLength * pivotAngle.getCos()
        + AnglerConstants.kDriverLength * getDrivenAngle(pivotAngle).getCos();
  }

  public static Rotation2d getJunctionAngle(Rotation2d pivotAngle) {
    return Rotation2d.fromRadians(Math.PI).minus(pivotAngle).minus(getDrivenAngle(pivotAngle));
  }

  public static Rotation2d getDrivenAngle(Rotation2d pivotAngle) {
    return Rotation2d.fromRadians(
        Math.asin(
            (AnglerConstants.kPivotLength / AnglerConstants.kDriverLength) * pivotAngle.getSin()));
  }

  public static Rotation2d getPerpendicularAngleDifference(Rotation2d pivotAngle) {
    Rotation2d exteriorToJunctionAngle =
        Rotation2d.fromRadians(Math.PI).minus(getJunctionAngle(pivotAngle));
    Rotation2d perpendicularAngle =
        Rotation2d.fromRadians(Math.PI / 2).minus(exteriorToJunctionAngle);

    return perpendicularAngle;
  }

  public static double scaleVoltage(Rotation2d pivotAngle) {
    return 1 / getPerpendicularAngleDifference(pivotAngle).getCos();
  }

  public static double getGravityUnitVector(Rotation2d pivotAngle) {
    return getPerpendicularAngleDifference(pivotAngle).getCos();
  }
}
