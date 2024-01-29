package frc.robot.subsystems.shooter.Flywheel;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterWheelSimIO implements ShooterWheelIO {
  private FlywheelSim flywheelMotorLeft =
      new FlywheelSim(DCMotor.getNEO(1), 
      ShooterWheelConstants.kGearing, 
      ShooterWheelConstants.kJKgMetersSquared, 
      VecBuilder.fill(0.1));
  private FlywheelSim flywheelMotorRight =
      new FlywheelSim(DCMotor.getNEO(1), 
      ShooterWheelConstants.kGearing, 
      ShooterWheelConstants.kJKgMetersSquared, 
      VecBuilder.fill(0.1));

  private double flywheelAppliedVoltsLeft = 0;
  private double flywheelVelocityMeasuredMPSLeft = 0;
  private double flywheelVelocitySetpointMPSLeft = 0;

  private double flywheelAppliedVoltsRight = 0;
  private double flywheelVelocityMeasuredMPSRight = 0;
  private double flywheelVelocitySetpointMPSRight = 0;

  PIDController flywheelPIDLeft = new PIDController(0.1, 0, 0);
  SimpleMotorFeedforward flywheelFeedforwardLeft = new SimpleMotorFeedforward(0.0, 0.1, 0.05);
  SlewRateLimiter flywheelRateLimiterLeft = new SlewRateLimiter(ShooterWheelConstants.kFlywheelRateLimitLeft);

  PIDController flywheelPIDRight = new PIDController(0.1, 0, 0);
  SimpleMotorFeedforward flywheelFeedforwardRight = new SimpleMotorFeedforward(0.0, 0.1, 0.05);
  SlewRateLimiter flywheelRateLimiterRight = new SlewRateLimiter(ShooterWheelConstants.kFlywheelRateLimitRight);

  @Override
  public void updateInputs(ShooterWheelIOInputs inputs) {
    flywheelVelocityMeasuredMPSLeft =
        (flywheelMotorLeft.getAngularVelocityRPM() * ShooterWheelConstants.kCircumferenceM) / 60;
    inputs.flywheelVelocityMPSLeft = flywheelVelocityMeasuredMPSLeft;
    inputs.flywheelAppliedVoltsLeft = flywheelAppliedVoltsLeft;
    inputs.flywheelCurrentAmpsLeft = new double[] {flywheelMotorLeft.getCurrentDrawAmps()};
    inputs.flywheelVelocityMPSLeftSetpoint = flywheelVelocitySetpointMPSLeft;

    flywheelVelocityMeasuredMPSRight =
        (flywheelMotorRight.getAngularVelocityRPM() * ShooterWheelConstants.kCircumferenceM) / 60;
    inputs.flywheelVelocityMPSRight = flywheelVelocityMeasuredMPSRight;
    inputs.flywheelAppliedVoltsRight = flywheelAppliedVoltsRight;
    inputs.flywheelCurrentAmpsRight = new double[] {flywheelMotorRight.getCurrentDrawAmps()};
    inputs.flywheelVelocityMPSRightSetpoint = flywheelVelocitySetpointMPSRight;

    flywheelMotorLeft.update(0.02);
    flywheelMotorRight.update(0.02);
  }

  @Override
  public void setFlywheelsVoltsLeft(double volts) {
    flywheelAppliedVoltsLeft = volts;
    flywheelMotorLeft.setInputVoltage(volts);
  }

  @Override
  public void setFlywheelsVoltsRight(double volts) {
    flywheelAppliedVoltsRight = volts;
    flywheelMotorRight.setInputVoltage(volts);
  }

  @Override
  public void setFlywheelsVelocityLeft(double velocityMPS) {
    double velocityMPSLimited = flywheelRateLimiterLeft.calculate(velocityMPS);
    flywheelVelocitySetpointMPSLeft = velocityMPS;
    double PID = flywheelPIDLeft.calculate(flywheelVelocityMeasuredMPSLeft, velocityMPS);
    if (velocityMPSLimited - velocityMPS > 1e-1)
      PID += flywheelFeedforwardLeft.calculate(velocityMPS, ShooterWheelConstants.kFlywheelRateLimitLeft);
    else PID += flywheelFeedforwardLeft.calculate(velocityMPS);

    setFlywheelsVoltsLeft(PID);
  }

  @Override
  public void setFlywheelsVelocityRight(double velocityMPS) {
    double velocityMPSLimited = flywheelRateLimiterRight.calculate(velocityMPS);
    flywheelVelocitySetpointMPSRight = velocityMPS;
    double PID = flywheelPIDRight.calculate(flywheelVelocityMeasuredMPSRight, velocityMPS);
    if (velocityMPSLimited - velocityMPS > 1e-1)
      PID += flywheelFeedforwardRight.calculate(velocityMPS, ShooterWheelConstants.kFlywheelRateLimitRight);
    else PID += flywheelFeedforwardRight.calculate(velocityMPS);

    setFlywheelsVoltsLeft(PID);
  }
}
