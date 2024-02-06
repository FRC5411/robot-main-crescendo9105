package frc.robot.subsystems.shooter.Flywheel;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class ShooterWheelNEO implements ShooterWheelIO {
  private CANSparkMax shooter;
  private RelativeEncoder shooterEncoder;
  private SparkPIDController shooterPID;

  private double velocityMeasuredMPS = 0;
  private double velocitySetpointMPS = 0;
  private double velocityRateLimit = 0;

  private SimpleMotorFeedforward velocityFeedforward;
  private SlewRateLimiter velocityRateLimiter;

  public ShooterWheelNEO(
      int id, PIDController pidController, SimpleMotorFeedforward feedforward, double rateLimit) {
    shooter = new CANSparkMax(id, CANSparkMax.MotorType.kBrushless);

    this.velocityFeedforward = feedforward;
    this.velocityRateLimit = rateLimit;
    this.velocityRateLimiter = new SlewRateLimiter(this.velocityRateLimit);
    configMotor(shooter, pidController);
  }

  @Override
  public void updateInputs(ShooterWheelIOInputs inputs) {
    velocityMeasuredMPS = shooterEncoder.getVelocity();
    inputs.flywheelVelocityMPS = velocityMeasuredMPS;
    inputs.flywheelAppliedVolts = shooter.getBusVoltage();
    inputs.flywheelCurrentAmps = new double[] {0.0, 0.0};
    inputs.flywheelVelocityMPSSetpoint = velocitySetpointMPS;
  }

  @Override
  public void setFlywheelsVolts(double volts) {
    shooter.setVoltage(volts);
  }

  @Override
  public void setFlywheelsVelocity(double velocityMPS) {
    double acceleration = 0;
    double velocityMPSLimited = velocityRateLimiter.calculate(velocityMPS);
    if (velocityMPSLimited != velocityMPS) acceleration = velocityRateLimit;
    shooterPID.setReference(
        velocityMPS,
        ControlType.kVelocity,
        0,
        velocityFeedforward.calculate(velocityMPS, acceleration),
        ArbFFUnits.kVoltage);
  }

  private void configMotor(CANSparkMax motor, PIDController pidController) {
    motor.restoreFactoryDefaults();
    motor.clearFaults();
    motor.setSmartCurrentLimit(20);
    motor.setSecondaryCurrentLimit(20);
    motor.setInverted(false);
    motor.enableVoltageCompensation(12);
    motor.setCANTimeout(10);
    motor.setIdleMode(IdleMode.kBrake);

    shooterEncoder = motor.getEncoder();
    shooterEncoder.setVelocityConversionFactor(
        ShooterWheelConstants.kCircumferenceM * ShooterWheelConstants.kGearing / 60);
    shooterEncoder.setVelocityConversionFactor(
        ShooterWheelConstants.kCircumferenceM * ShooterWheelConstants.kGearing);

    shooterPID = motor.getPIDController();
    shooterPID.setP(ShooterWheelConstants.kP);
    shooterPID.setI(ShooterWheelConstants.kI);
    shooterPID.setD(ShooterWheelConstants.kD);
    shooterPID.setFeedbackDevice(shooterEncoder);

    motor.burnFlash();
  }
}
