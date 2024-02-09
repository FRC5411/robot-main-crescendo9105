// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.angler;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Class to represent the angler in simulation */
public class AnglerIOSim implements AnglerIO {
  private double LOOP_PERIOD_S = 0.02;

  // TODO Update as needed
  private SingleJointedArmSim anglerMotor =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          1.0,
          -0.057,
          3.0,
          Units.degreesToRadians(15.0),
          Units.degreesToRadians(75.0),
          false,
          Units.degreesToRadians(15));

  private double appliedVolts = 0.0;

  /** Create a new virtual implementation of the angler */
  public AnglerIOSim() {}

  @Override
  public void updateInputs(AnglerIOInputs inputs) {
    anglerMotor.update(LOOP_PERIOD_S);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(anglerMotor.getCurrentDrawAmps()));

    inputs.angleRadians = anglerMotor.getAngleRads();
    inputs.velocityRPS = anglerMotor.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.appliedCurrentAmps = new double[] {anglerMotor.getCurrentDrawAmps()};
    inputs.temperatureCelsius = new double[] {0.0};
  }

  @Override
  public void setVolts(double volts) {
    var adjustedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    anglerMotor.setInputVoltage(adjustedVolts);
  }
}
