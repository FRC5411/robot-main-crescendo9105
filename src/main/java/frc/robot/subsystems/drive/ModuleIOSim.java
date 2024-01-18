// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** A simulated swerve module */
public class ModuleIOSim implements ModuleIO {
  private final double LOOP_PERIOD_S = 0.02;

  // TODO Update values to reflect real hardare as needed
  private DCMotorSim driveMotor = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
  private DCMotorSim azimuthMotor = new DCMotorSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);

  private final Rotation2d INITIAL_ABSOLUTE_ANGLE = new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double driveAppliedVolts = 0.0;
  private double azimuthAppliedVolts = 0.0;

  // TODO Tune sim values as needed
  private PIDController driveController = new PIDController(0.1, 0.0, 0.0);
  private PIDController azimuthController = new PIDController(10.0, 0.0, 0.0);
  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
  private SimpleMotorFeedforward azimuthFeedforward = new SimpleMotorFeedforward(0.0, 0.0);

  public ModuleIOSim(int module) {}

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveMotor.update(LOOP_PERIOD_S);
    azimuthMotor.update(LOOP_PERIOD_S);

    inputs.drivePositionM = driveMotor.getAngularPositionRad();
    inputs.driveVelocityMPS = driveMotor.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(driveMotor.getCurrentDrawAmps())};

    inputs.azimuthAbsolutePosition =
        new Rotation2d(azimuthMotor.getAngularPositionRad()).plus(INITIAL_ABSOLUTE_ANGLE);
    inputs.azimuthPosition = new Rotation2d(azimuthMotor.getAngularPositionRad());
    inputs.azimuthVelocityRPS = azimuthMotor.getAngularVelocityRadPerSec();
    inputs.azimuthAppliedVolts = azimuthAppliedVolts;
    inputs.azimuthCurrentAmps = new double[] {Math.abs(azimuthMotor.getCurrentDrawAmps())};

    inputs.odometryDrivePositionR = new double[] {inputs.drivePositionM};
    inputs.odometryAzimuthPositions = new Rotation2d[] {inputs.azimuthPosition};
  }

  @Override
  public void setDriveVolts(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveMotor.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setAzimuthVolts(double volts) {
    azimuthAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    azimuthMotor.setInputVoltage(azimuthAppliedVolts);
  }

  @Override
  public void setVelocity(double velocityRPS) {
    setDriveVolts(
        driveController.calculate(driveMotor.getAngularVelocityRadPerSec(), velocityRPS)
            + driveFeedforward.calculate(velocityRPS));
  }

  @Override
  public void setAngle(double angleR) {
    // Feedforward will help drive the motor based on the PID error
    setAzimuthVolts(
        azimuthController.calculate(azimuthMotor.getAngularPositionRad(), angleR)
            + azimuthFeedforward.calculate(Math.signum(azimuthController.getPositionError())));
  }
}
