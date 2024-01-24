package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.shooter.LeadScrewArm.ScrewArmConstants;

public class ScrewArmKinematics {
    public static double getLengthAlongScrew(Rotation2d pivotAngle) {
        return 
            ScrewArmConstants.kScrewArmPivotLength * pivotAngle.getCos() +
            ScrewArmConstants.kScrewArmDriverLength * getDrivenAngle(pivotAngle).getCos();
    }

    public static Rotation2d getJunctionAngle(Rotation2d pivotAngle) {
        return Rotation2d.fromRadians(Math.PI).minus(pivotAngle).minus(getDrivenAngle(pivotAngle));
    }

    public static Rotation2d getDrivenAngle(Rotation2d pivotAngle) {
        return Rotation2d.fromRadians( 
                Math.asin( (ScrewArmConstants.kScrewArmPivotLength / 
                ScrewArmConstants.kScrewArmDriverLength) * pivotAngle.getSin() ) );
    }

    public static Rotation2d getPerpendicularAngleDifference(Rotation2d pivotAngle) {
        Rotation2d exteriorToJunctionAngle = getDrivenAngle(pivotAngle).plus(pivotAngle);
        Rotation2d perpendicularAngle = Rotation2d.fromRadians(Math.PI / 2).minus(pivotAngle);

        return exteriorToJunctionAngle.minus(perpendicularAngle);
    }

    public static double scaleVoltage(Rotation2d pivotAngle) {
        return 1 / getPerpendicularAngleDifference(pivotAngle).getCos();
    }

    public static double getGravityVector(Rotation2d pivotAngle) {
        return ScrewArmConstants.kScrewArmKG * getPerpendicularAngleDifference(pivotAngle).getCos();
    }
}