// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.yoshivator.manipulator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Class to represent the manipulator mechanism in simulation */
public class ManipulatorIOSim implements ManipulatorIO {
  private final double LOOP_PERIOD_S = 0.02;
  private final double PIVOT_GEARING = (64.0 / 1.0) * (3.0 / 1.0);
  private final double FLYWHEEL_GEARING = 5.0 / 1.0;

  private SingleJointedArmSim pivotMotor =
      new SingleJointedArmSim(
          DCMotor.getNEO(1), PIVOT_GEARING, 0.002, 0.5, -15.0, 120.0, true, 0.0);
  private FlywheelSim flywheelMotor =
      new FlywheelSim(DCMotor.getNeo550(1), FLYWHEEL_GEARING, 0.002);

  private double pivotAppliedVolts = 0.0;
  private double flywheelAppliedVolts = 0.0;

  /** Create a new virtual implementation of the manipulator */
  public ManipulatorIOSim() {}

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    pivotMotor.update(LOOP_PERIOD_S);
    flywheelMotor.update(LOOP_PERIOD_S);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            pivotMotor.getCurrentDrawAmps() + flywheelMotor.getCurrentDrawAmps()));

    inputs.pivotPosition = Rotation2d.fromRadians(pivotMotor.getAngleRads());
    inputs.pivotAppliedVolts = pivotAppliedVolts;
    inputs.pivotInternalVolts = 0.0;
    inputs.pivotAppliedCurrentAmps = new double[] {pivotMotor.getCurrentDrawAmps()};
    inputs.pivotTemperatureCelsius = new double[] {0.0};

    inputs.flywheelVelocityRPM = flywheelMotor.getAngularVelocityRPM();
    inputs.flywheelAppliedVolts = flywheelAppliedVolts;
    inputs.flywheelInternalVolts = 0.0;
    inputs.flywheelAppliedCurrentAmps = new double[] {flywheelMotor.getCurrentDrawAmps()};
    inputs.flywheelTemperatureCelsius = new double[] {0.0};
  }

  @Override
  public void setPivotVolts(double volts) {
    pivotAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    pivotMotor.setInputVoltage(pivotAppliedVolts);
  }

  @Override
  public void setFlywheelVolts(double volts) {
    flywheelAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    flywheelMotor.setInputVoltage(flywheelAppliedVolts);
  }
}
