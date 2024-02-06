package frc.robot.subsystems.shooter.Angler;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface AnglerIO {
  @AutoLog
  public static class AnglerInputs {
    public Rotation2d anglerAngle = new Rotation2d();
    public Rotation2d anglerAngleSetpoint = new Rotation2d();
    public Rotation2d anglerAngleGoal = new Rotation2d();
    public double anglerAppliedVolts = 0.0;
    public double anglerCurrentAmps = 0.0;
    public double anglerMotorTempC = 0.0;
    public boolean anglerAtGoal = false;
    public boolean anglerAtSetpoint = false;
  }

  public default void updateInputs(AnglerInputs inputs) {}

  public default void setAnglerVolts(double volts) {}

  public default void setGoal(Rotation2d goal) {}

  public default void initPID() {}

  public default void executePID() {}
}
