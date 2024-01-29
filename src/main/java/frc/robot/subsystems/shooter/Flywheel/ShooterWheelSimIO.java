package frc.robot.subsystems.shooter.Flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterWheelSimIO implements ShooterWheelIO {
  private FlywheelSim flywheelMotor =
      new FlywheelSim(DCMotor.getNEO(1), 
      ShooterWheelConstants.kGearing, 
      ShooterWheelConstants.kJKgMetersSquared, 
      VecBuilder.fill(0.1));

  private double appliedVolts = 0;
  private double velocityMeasuredMPS = 0;
  private double velocitySetpointMPS = 0;
  private double velocityRateLimit = 0.5;

  PIDController velocityController;
  SimpleMotorFeedforward velocityFeedforward;
  SlewRateLimiter velocityRateLimiter;

  public ShooterWheelSimIO(
    PIDController flywheelPID, SimpleMotorFeedforward flywheelFeedforward, double flywheelRateLimit) {
    this.velocityController = flywheelPID;
    this.velocityFeedforward = flywheelFeedforward;
    this.velocityRateLimit = flywheelRateLimit;
    this.velocityRateLimiter = new SlewRateLimiter(this.velocityRateLimit);
  }

  @Override
  public void updateInputs(ShooterWheelIOInputs inputs) {
    velocityMeasuredMPS =
        (flywheelMotor.getAngularVelocityRPM() * ShooterWheelConstants.kCircumferenceM) / 60;
    inputs.flywheelVelocityMPS = velocityMeasuredMPS;
    inputs.flywheelAppliedVolts = appliedVolts;
    inputs.flywheelCurrentAmps = new double[] {flywheelMotor.getCurrentDrawAmps()};
    inputs.flywheelVelocityMPSSetpoint = velocitySetpointMPS;

    flywheelMotor.update(0.2);
  }

  @Override
  public void setFlywheelsVolts(double volts) {
    appliedVolts = volts;
    flywheelMotor.setInputVoltage( MathUtil.clamp( volts, -12, 12 ));
  }

  @Override
  public void setFlywheelsVelocity(double velocityMPS) {
    double velocityMPSLimited = velocityRateLimiter.calculate(velocityMPS);
    velocitySetpointMPS = velocityMPS;
    double PID = velocityController.calculate(velocityMeasuredMPS, velocityMPS);
    if (velocityMPSLimited - velocityMPS > 1e-1)
      PID += velocityFeedforward.calculate(velocityMPS, ShooterWheelConstants.kFlywheelRateLimit);
    else PID += velocityFeedforward.calculate(velocityMPS);

    setFlywheelsVolts(PID);
  }
}
