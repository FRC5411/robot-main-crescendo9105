package frc.robot.subsystems.shooter.LeadScrewArm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

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
    public boolean screwArmAtGoal = false;
    public boolean screwArmAtSetpoint = false;
  }

  public default void updateInputs(ScrewArmInputs inputs) {}

  public default void setScrewArmVolts(double volts) {}

  public default void setGoal(Rotation2d goal) {}

  public default void initPID() {}

  public default void executePID() {}

  // Dangerous, but don't know an alternative
  public default boolean atGoal() {
    return false;
  }
}
