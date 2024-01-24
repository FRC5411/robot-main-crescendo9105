package frc.robot.subsystems.shooter.Flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterWheelIO {
    @AutoLog
    public static class ShooterWheelIOInputs {
        public double flywheelVelocityMPSLeft = 0.0;
        public double flywheelAppliedVoltsLeft = 0.0;
        public double[] flywheelCurrentAmpsLeft = new double[] {};
        public double flywheelVelocityMPSLeftSetpoint = 0.0;

        public double flywheelVelocityMPSRight = 0.0;
        public double flywheelAppliedVoltsRight = 0.0;
        public double[] flywheelCurrentAmpsRight = new double[] {};
        public double flywheelVelocityMPSRightSetpoint = 0.0;
    }

    public default void updateInputs(ShooterWheelIOInputs inputs) {}

    public default void setFlywheelsVoltsLeft(double volts) {}

    public default void setFlywheelsVoltsRight(double volts) {}

    public default void setFlywheelsVolts(double volts) {
        setFlywheelsVoltsLeft(volts);
        setFlywheelsVoltsRight(volts);
    }

    public default void setFlywheelsVelocityLeft(double velocityRPM) {}

    public default void setFlywheelsVelocityRight(double velocityRPM) {}

    public default void setFlywheelsVelocity(double velocityRPM) {
        setFlywheelsVelocityLeft(velocityRPM);
        setFlywheelsVelocityRight(velocityRPM);
    }
}