// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Class to represent the mechanism in simulation */
public class ClimbIOSim implements ClimbIO {
  private double LOOP_PERIOD_S = 0.02;

  // TODO Update args as needed
  private SingleJointedArmSim leftMotor =
      new SingleJointedArmSim(
          DCMotor.getNEO(1), 125.0, 0.005, Units.inchesToMeters(12.5), 0.0, 2.0, true, 0.0);
  private SingleJointedArmSim rightMotor =
      new SingleJointedArmSim(
          DCMotor.getNEO(1), 125.0, 0.005, Units.inchesToMeters(12.5), 0.0, 2.0, true, 0.0);

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  /** Create a new virtual implementation of the climb */
  public ClimbIOSim() {}

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    leftMotor.update(LOOP_PERIOD_S);
    rightMotor.update(LOOP_PERIOD_S);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(leftMotor.getCurrentDrawAmps()));
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(rightMotor.getCurrentDrawAmps()));

    inputs.leftAngleRadians = leftMotor.getAngleRads();
    inputs.leftVelocityRPS = leftMotor.getVelocityRadPerSec();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = new double[] {leftMotor.getCurrentDrawAmps()};
    inputs.leftTemperatureCelsius = new double[] {0.0};

    inputs.rightAngleRadians = rightMotor.getAngleRads();
    inputs.rightVelocityRPS = rightMotor.getVelocityRadPerSec();
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = new double[] {rightMotor.getCurrentDrawAmps()};
    inputs.rightTemperatureCelsius = new double[] {0.0};
  }

  @Override
  public void setLeftVolts(double volts) {
    leftAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    leftMotor.setInputVoltage(leftAppliedVolts);
    leftMotor.setInput(leftAppliedVolts / 12.0);
  }

  @Override
  public void setRightVolts(double volts) {
    rightAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    rightMotor.setInputVoltage(rightAppliedVolts);
    rightMotor.setInput(rightAppliedVolts / 12.0);
  }
}