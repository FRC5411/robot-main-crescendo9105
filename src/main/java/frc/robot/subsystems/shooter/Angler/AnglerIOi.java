// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.angler;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for representing the hardware */
public interface AnglerIOi {
  @AutoLog
  public static class AnglerIOInputsI {
    public Rotation2d anglerAngle = new Rotation2d();
    public Rotation2d anglerAngleSetpoint = new Rotation2d();
    public Rotation2d anglerAngleGoal = new Rotation2d();
    public double anglerAppliedVolts = 0.0;
    public double anglerCurrentAmps = 0.0;
    public double anglerMotorTempC = 0.0;
    public boolean anglerAtGoal = false;
    public boolean anglerAtSetpoint = false;
  }

  public default void updateInputs(AnglerIOInputsI inputs) {}

  public default void setAnglerVolts(double volts) {}

  public default void setGoal(Rotation2d goal) {}

  public default void initPID() {}

  public default void executePID() {}
}
