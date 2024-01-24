package frc.robot.subsystems.shooter.LeadScrewArm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ScrewArmIO {
    @AutoLog
    public static class ScrewArmInputs {
        public Rotation2d screwArmDegrees = new Rotation2d();
        public Rotation2d screwArmDegreesSetpoint = new Rotation2d();
        public double screwArmPositionMeters = 0.0;
        public double screwArmVelocityMeters = 0.0;
        public double screwArmAppliedVolts = 0.0;
        public double screwArmCurrentAmps = 0.0;
        public double screwArmMotorTempC = 0.0;
    }

    public default void updateInputs(ScrewArmInputs inputs) {}

    public default void setScrewArmVolts(double volts) {}

    public default void setGoal(Rotation2d goal) {}

    public default void initPID(Rotation2d measured) {}

    public default void executePID(Rotation2d setpoint) {}
}
