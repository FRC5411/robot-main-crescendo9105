// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.angler;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Class to represent the angler mechanism in simulation */
public class AnglerIOSim implements AnglerIO {
  private final double LOOP_PERIOD_S = 0.02;
  private final double GEARING = 1.0 / 1.0;

  private SingleJointedArmSim anglerMotor =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          GEARING,
          0.167248163371,
          0.4826,
          Rotation2d.fromDegrees(-180.0).getRadians(),
          Rotation2d.fromDegrees(180.0).getRadians(),
          false,
          Rotation2d.fromDegrees(45.0).getRadians(),
          VecBuilder.fill(0.0001));

  private double appliedVolts = 0.0;

  /** Create a new virtual implementation of the angler */
  public AnglerIOSim() {}

  @Override
  public void updateInputs(AnglerIOInputs inputs) {
    anglerMotor.update(LOOP_PERIOD_S);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(anglerMotor.getCurrentDrawAmps()));

    inputs.anglerAbsolutePosition = Rotation2d.fromRadians(anglerMotor.getAngleRads());
    inputs.anglerRelativePosition = Rotation2d.fromRadians(anglerMotor.getAngleRads());
    inputs.anglerDutyCycleFrequency = 955;
    inputs.anglerVelocityRadiansPerSecond = anglerMotor.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.internalVolts = appliedVolts;
    inputs.appliedCurrentAmps = new double[] {anglerMotor.getCurrentDrawAmps()};
    inputs.temperatureCelsius = new double[] {0.0};
  }

  @Override
  public void setVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    anglerMotor.setInputVoltage(appliedVolts);
  }
}
