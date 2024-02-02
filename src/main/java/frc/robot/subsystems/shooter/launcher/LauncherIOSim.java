// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/** Class to represent the launcher in simulation */
public class LauncherIOSim implements LauncherIO {
  private double LOOP_PERIOD_S = 0.02;

  private FlywheelSim leftMotor = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 1.0);
  private FlywheelSim rightMotor = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 1.0);

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  /** Create a new virtual implementation of the launcher */
  public LauncherIOSim() {}

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    leftMotor.update(LOOP_PERIOD_S);
    rightMotor.update(LOOP_PERIOD_S);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(leftMotor.getCurrentDrawAmps()));
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(rightMotor.getCurrentDrawAmps()));

    inputs.leftAngleRadians = 0.0;
    inputs.leftVelocityRPS = leftMotor.getAngularVelocityRadPerSec();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftAppliedCurrentAmps = new double[] {leftMotor.getCurrentDrawAmps()};
    inputs.leftTemperatureCelsius = new double[] {0.0};

    inputs.rightAngleRadians = 0.0;
    inputs.rightVelocityRPS = rightMotor.getAngularVelocityRadPerSec();
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightAppliedCurrentAmps = new double[] {rightMotor.getCurrentDrawAmps()};
    inputs.rightTemperatureCelsius = new double[] {0.0};
  }

  @Override
  public void setVolts(double leftVolts, double rightVolts) {
    leftAppliedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
    rightAppliedVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);

    leftMotor.setInputVoltage(leftVolts);
    rightMotor.setInputVoltage(rightVolts);
  }
}
