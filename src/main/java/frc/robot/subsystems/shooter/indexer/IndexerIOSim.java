// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/** Class to represent the indexer in simulation */
public class IndexerIOSim implements IndexerIO {
  private double LOOP_PERIOD_S = 0.02;

  private FlywheelSim indexerMotor = new FlywheelSim(DCMotor.getNeo550(1), 1.0, 1.0);

  private double appliedVolts = 0.0;

  /** Create a new virtual implementation of the indexer */
  public IndexerIOSim() {}

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // TODO Update as needed
    indexerMotor.update(LOOP_PERIOD_S);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(indexerMotor.getCurrentDrawAmps()));

    inputs.angleRadians = 0.0;
    inputs.velocityRPM = indexerMotor.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.appliedCurrentAmps = new double[] {indexerMotor.getCurrentDrawAmps()};
    inputs.temperatureCelsius = new double[] {0.0};
  }

  @Override
  public void setVolts(double volts) {
    var adjustedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    indexerMotor.setInputVoltage(adjustedVolts);
  }
}
