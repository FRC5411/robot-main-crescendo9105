// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.utils.debugging.LoggedTunableNumber;

/** Class to represent the indexer mechanism in simulation */
public class IndexerIOSim implements IndexerIO {
  private final double LOOP_PERIOD_S = 0.02;
  private final double GEARING = 5.0 / 1.0;

  private FlywheelSim indexerMotor =
      new FlywheelSim(DCMotor.getNEO(1), GEARING, 0.0002, VecBuilder.fill(0.01));

  // 1 = true, 0 = false
  private LoggedTunableNumber beamBreakSensorSim = new LoggedTunableNumber("Indexer/IR/Value", 0.0);

  private double appliedVolts = 0.0;

  /** Create a new virtual implementation of the indexer */
  public IndexerIOSim() {}

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    indexerMotor.update(LOOP_PERIOD_S);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(indexerMotor.getCurrentDrawAmps()));

    inputs.indexerVelocityRPM = indexerMotor.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.internalVolts = appliedVolts;
    inputs.appliedCurrentAmps = new double[] {indexerMotor.getCurrentDrawAmps()};
    inputs.temperatureCelsius = new double[] {0.0};
    inputs.isBeamBroken = (beamBreakSensorSim.get() == 1.0) ? true : false;
  }

  @Override
  public void setVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    indexerMotor.setInputVoltage(appliedVolts);
  }
}
