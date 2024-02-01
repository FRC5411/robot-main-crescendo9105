// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Class to represent the indexer in simulation */
public class IndexerIOSim implements IndexerIO {
  private double LOOP_PERIOD_S = 0.02;

  private SingleJointedArmSim indexerMotor =
      new SingleJointedArmSim(
          DCMotor.getNeo550(1),
          1.0,
          0.0,
          3.0,
          Units.degreesToRadians(0.0),
          Units.degreesToRadians(360.0),
          false,
          Units.degreesToRadians(0.0));

  private double appliedVolts = 0.0;

  /** Create a new virtual implementation of the indexer */
  public IndexerIOSim() {}

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    indexerMotor.update(LOOP_PERIOD_S);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(indexerMotor.getCurrentDrawAmps()));

    inputs.angleRadians = indexerMotor.getAngleRads();
    inputs.velocityRPS = indexerMotor.getVelocityRadPerSec();
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
