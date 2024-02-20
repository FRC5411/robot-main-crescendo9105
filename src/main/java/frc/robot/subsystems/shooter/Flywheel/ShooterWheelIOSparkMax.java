package frc.robot.subsystems.shooter.flywheel;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.utils.debugging.LoggedTunableNumber;

public class ShooterWheelIOSparkMax implements ShooterWheelIO {
  private CANSparkMax shooter;
  private RelativeEncoder shooterEncoder;
  private SparkPIDController velocityController;

  private double velocityMeasuredMPS = 0;
  private double velocitySetpointMPS = 0;
  private double velocityRateLimit = 0;

  private SimpleMotorFeedforward velocityFeedforward;
  private SlewRateLimiter velocityRateLimiter;

  private LoggedTunableNumber shooterWheelControlP;
  private LoggedTunableNumber shooterWheelControlI;
  private LoggedTunableNumber shooterWheelControlD;

  public ShooterWheelIOSparkMax(
      int id,
      PIDController pidController,
      SimpleMotorFeedforward feedforward,
      double rateLimit,
      String key) {
    shooter = new CANSparkMax(id, CANSparkMax.MotorType.kBrushless);

    this.velocityFeedforward = feedforward;
    this.velocityRateLimit = rateLimit;
    this.velocityRateLimiter = new SlewRateLimiter(this.velocityRateLimit);
    configMotor(shooter, pidController);

    this.shooterWheelControlP =
        new LoggedTunableNumber("ShooterWheel/P" + key, velocityController.getP());
    this.shooterWheelControlI =
        new LoggedTunableNumber("ShooterWheel/I" + key, velocityController.getI());
    this.shooterWheelControlD =
        new LoggedTunableNumber("ShooterWheel/D" + key, velocityController.getD());
  }

  @Override
  public void updateInputs(ShooterWheelIOInputs inputs) {
    velocityMeasuredMPS = shooterEncoder.getVelocity();
    inputs.flywheelVelocityMPS = velocityMeasuredMPS;
    inputs.flywheelAppliedVolts = shooter.getBusVoltage();
    inputs.flywheelCurrentAmps = new double[] {0.0, 0.0};
    inputs.flywheelVelocityMPSSetpoint = velocitySetpointMPS;

    if (shooterWheelControlP.hasChanged(hashCode())
        || shooterWheelControlI.hasChanged(hashCode())
        || shooterWheelControlD.hasChanged(hashCode())) {
      velocityController.setP(shooterWheelControlP.get());
      velocityController.setI(shooterWheelControlI.get());
      velocityController.setD(shooterWheelControlD.get());
    }
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
    velocityController.setReference(
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

    velocityController = motor.getPIDController();
    velocityController.setP(ShooterWheelConstants.kP);
    velocityController.setI(ShooterWheelConstants.kI);
    velocityController.setD(ShooterWheelConstants.kD);
    velocityController.setFeedbackDevice(shooterEncoder);

    motor.burnFlash();
  }
}
