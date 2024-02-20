package frc.robot.subsystems.shooter.Flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterWheelIO {
  @AutoLog
  public static class ShooterWheelIOInputs {
    public double flywheelVelocityMPS = 0.0;
    public double flywheelAppliedVolts = 0.0;
    public double[] flywheelCurrentAmps = new double[] {};
    public double flywheelVelocityMPSSetpoint = 0.0;
  }

  public default void updateInputs(ShooterWheelIOInputs inputs) {}

  public default void setFlywheelsVolts(double volts) {}

  public default void setFlywheelsVelocity(double velocityRPM) {}
}
