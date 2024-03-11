// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public Rotation2d leftPosition = new Rotation2d();
    public double leftVelocityRPS = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftInternalVolts = 0.0;
    public double[] leftCurrentAmps = new double[] {0.0};
    public double[] leftTemperatureCelsius = new double[] {0.0};

    public Rotation2d rightPosition = new Rotation2d();
    public double rightVelocityRPS = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightInternalVolts = 0.0;
    public double[] rightCurrentAmps = new double[] {0.0};
    public double[] rightTemperatureCelsius = new double[] {0.0};
  }

  /** Update the inputs from the sensors */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /** Set the voltage of the left climb motor */
  public default void setLeftVolts(double volts) {}

  /** Set the voltage of the right climb motor */
  public default void setRightVolts(double volts) {}
}
