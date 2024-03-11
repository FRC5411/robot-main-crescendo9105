// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ClimbIOSim implements ClimbIO {
  
  private SingleJointedArmSim leftMotor;
  private double leftAppliedVolts;
  
  private SingleJointedArmSim rightMotor;
  private double rightAppliedVolts;
  
  private final DCMotor motorInGearbox  = DCMotor.getNEO(1);
  private final double gearing          = 125.0;
  private final double jKgMetersSquared = 0.005;
  private final double armLengthMeters  = Units.inchesToMeters(12.5);
  private final double minAngle         = -Math.PI;
  private final double maxAngle         = Math.PI;
  private final boolean simulateGravity = true;
  private final double startingAngle    = Math.toRadians(180.0);  
  private final double loopPeriodSec    = 0.02;

  public ClimbIOSim() {
    leftAppliedVolts = 0.0;
    rightAppliedVolts = 0.0;
    leftMotor = new SingleJointedArmSim(motorInGearbox, gearing, jKgMetersSquared, armLengthMeters, minAngle, maxAngle, simulateGravity, startingAngle);
    rightMotor = new SingleJointedArmSim(motorInGearbox, gearing, jKgMetersSquared, armLengthMeters, minAngle, maxAngle, simulateGravity, startingAngle);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    leftMotor.update(loopPeriodSec);
    rightMotor.update(loopPeriodSec);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(leftMotor.getCurrentDrawAmps()));
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(rightMotor.getCurrentDrawAmps()));

    inputs.leftPosition = Rotation2d.fromRadians(leftMotor.getAngleRads());
    inputs.leftVelocityRPS = leftMotor.getVelocityRadPerSec();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftInternalVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = new double[] {leftMotor.getCurrentDrawAmps()};
    inputs.leftTemperatureCelsius = new double[] {0.0};

    inputs.rightPosition = Rotation2d.fromRadians(rightMotor.getAngleRads());
    inputs.rightVelocityRPS = rightMotor.getVelocityRadPerSec();
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightInternalVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = new double[] {rightMotor.getCurrentDrawAmps()};
    inputs.rightTemperatureCelsius = new double[] {0.0};
  }

  @Override
  public void setLeftVolts(double volts) {
    leftAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    leftMotor.setInputVoltage(leftAppliedVolts);
  }

  @Override
  public void setRightVolts(double volts) {
    rightAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    rightMotor.setInputVoltage(rightAppliedVolts);
  }
}
