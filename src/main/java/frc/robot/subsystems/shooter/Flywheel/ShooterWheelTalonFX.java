package frc.robot.subsystems.shooter.Flywheel;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class ShooterWheelTalonFX implements ShooterWheelIO {
  private TalonFX flywheelMotorLeft = new TalonFX(ShooterWheelConstants.kLeftMotorID);
  private TalonFX flywheelMotorRight = new TalonFX(ShooterWheelConstants.kRightMotorID);

  private double flywheelVelocitySetpointMPSLeft = 0;
  private double flywheelRateLimitLeft = 0.5;

  private double flywheelVelocitySetpointMPSRight = 0;
  private double flywheelRateLimitRight = 0.5;

  private VelocityVoltage flywheelVelocityLeft = new VelocityVoltage(0.0);
  private SlewRateLimiter flywheelRateLimiterLeft = new SlewRateLimiter(0.5);

  private VelocityVoltage flywheelVelocityRight = new VelocityVoltage(0.0);
  private SlewRateLimiter flywheelRateLimiterRight = new SlewRateLimiter(0.5);

  public ShooterWheelTalonFX() {
    flywheelMotorLeft.setInverted(true);
    flywheelMotorRight.setInverted(false);
  }

  @Override
  public void updateInputs(ShooterWheelIOInputs inputs) {
    inputs.flywheelVelocityMPSLeft = 
      flywheelMotorLeft.getVelocity().getValueAsDouble() * ShooterWheelConstants.kCircumferenceM;
    inputs.flywheelAppliedVoltsLeft = flywheelMotorLeft.getMotorVoltage().getValueAsDouble();
    inputs.flywheelCurrentAmpsLeft = new double[] { flywheelMotorLeft.getStatorCurrent().getValueAsDouble() };
    inputs.flywheelVelocityMPSLeftSetpoint = flywheelVelocitySetpointMPSLeft;

    inputs.flywheelVelocityMPSRight = 
      flywheelMotorRight.getVelocity().getValueAsDouble() * ShooterWheelConstants.kCircumferenceM;
    inputs.flywheelAppliedVoltsRight = flywheelMotorRight.getMotorVoltage().getValueAsDouble();
    inputs.flywheelCurrentAmpsRight = new double[] { flywheelMotorRight.getStatorCurrent().getValueAsDouble() } ;
    inputs.flywheelVelocityMPSRightSetpoint = flywheelVelocitySetpointMPSRight;
  }

  @Override
  public void setFlywheelsVoltsLeft(double volts) {
      flywheelMotorLeft.setVoltage(volts);
  }

  @Override
  public void setFlywheelsVoltsRight(double volts) {
    flywheelMotorRight.setVoltage(volts);
  }

  @Override
  public void setFlywheelsVelocityLeft(double velocityMPS) {
    double newVelocityMPS = flywheelRateLimiterLeft.calculate(velocityMPS);
    double acceleration = 0;
    if(newVelocityMPS != velocityMPS) acceleration = flywheelRateLimitLeft;
    flywheelVelocitySetpointMPSLeft = velocityMPS;
    flywheelMotorLeft.setControl(
      flywheelVelocityLeft.withVelocity( 
        flywheelVelocitySetpointMPSLeft / ShooterWheelConstants.kCircumferenceM )
      .withAcceleration(acceleration) );
  }

  @Override
  public void setFlywheelsVelocityRight(double velocityMPS) {
    double newVelocityMPS = flywheelRateLimiterRight.calculate(velocityMPS);
    double acceleration = 0;
    if(newVelocityMPS != velocityMPS) acceleration = flywheelRateLimitRight;
    flywheelVelocitySetpointMPSRight = velocityMPS;
    flywheelMotorRight.setControl(
      flywheelVelocityRight.withVelocity( 
        flywheelVelocitySetpointMPSRight / ShooterWheelConstants.kCircumferenceM )
      .withAcceleration(acceleration) );
  }
}
