package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.utils.debugging.LoggedTunableNumber;

public class ShooterWheelTalonFX implements ShooterWheelIO {
  private TalonFX flywheelMotor;

  private double flywheelVelocitySetpointMPS = 0;
  private double velocityRateLimit;

  private VelocityVoltage flywheelVelocity = new VelocityVoltage(0.0);
  private SlewRateLimiter velocityRateLimiter;

  private LoggedTunableNumber shooterWheelControlP;
  private LoggedTunableNumber shooterWheelControlI;
  private LoggedTunableNumber shooterWheelControlD;
  private LoggedTunableNumber shooterWheelControlS;
  private LoggedTunableNumber shooterWheelControlV;
  private LoggedTunableNumber shooterWheelControlA;

  public ShooterWheelTalonFX(
      int motorID,
      boolean invert,
      PIDController velocity,
      SimpleMotorFeedforward velocityFeedforward,
      double flywheelRateLimit,
      String key) {
    configMotor(motorID, invert, velocity, velocityFeedforward);
    this.velocityRateLimit = flywheelRateLimit;
    this.velocityRateLimiter = new SlewRateLimiter(this.velocityRateLimit);

    this.shooterWheelControlP =
        new LoggedTunableNumber("ShooterWheel/P" + key, ShooterWheelConstants.kP);
    this.shooterWheelControlI =
        new LoggedTunableNumber("ShooterWheel/I" + key, ShooterWheelConstants.kI);
    this.shooterWheelControlD =
        new LoggedTunableNumber("ShooterWheel/D" + key, ShooterWheelConstants.kD);
    this.shooterWheelControlS =
        new LoggedTunableNumber("ShooterWheel/S" + key, ShooterWheelConstants.kS);
    this.shooterWheelControlV =
        new LoggedTunableNumber("ShooterWheel/V" + key, ShooterWheelConstants.kV);
    this.shooterWheelControlA =
        new LoggedTunableNumber("ShooterWheel/A" + key, ShooterWheelConstants.kA);
  }

  @Override
  public void updateInputs(ShooterWheelIOInputs inputs) {
    inputs.flywheelVelocityMPS =
        flywheelMotor.getVelocity().getValueAsDouble() * ShooterWheelConstants.kCircumferenceM;
    inputs.flywheelAppliedVolts = flywheelMotor.getMotorVoltage().getValueAsDouble();
    inputs.flywheelCurrentAmps = new double[] {flywheelMotor.getStatorCurrent().getValueAsDouble()};
    inputs.flywheelVelocityMPSSetpoint = flywheelVelocitySetpointMPS;

    if (shooterWheelControlP.hasChanged(hashCode())
        || shooterWheelControlI.hasChanged(hashCode())
        || shooterWheelControlD.hasChanged(hashCode())
        || shooterWheelControlS.hasChanged(hashCode())
        || shooterWheelControlV.hasChanged(hashCode())
        || shooterWheelControlA.hasChanged(hashCode())) {
      Slot0Configs config = new Slot0Configs();
      config.kP = shooterWheelControlP.get();
      config.kI = shooterWheelControlI.get();
      config.kD = shooterWheelControlD.get();
      config.kS = shooterWheelControlS.get();
      config.kV = shooterWheelControlV.get();
      config.kA = shooterWheelControlA.get();
      flywheelMotor.getConfigurator().apply(config, 0.1);
    }
  }

  @Override
  public void setFlywheelsVolts(double volts) {
    flywheelMotor.setVoltage(volts);
  }

  @Override
  public void setFlywheelsVelocity(double velocityMPS) {
    double newVelocityMPS = velocityRateLimiter.calculate(velocityMPS);
    double acceleration = 0;
    if (newVelocityMPS != velocityMPS) acceleration = velocityRateLimit;
    flywheelVelocitySetpointMPS = velocityMPS;

    flywheelMotor.setControl(
        flywheelVelocity
            .withVelocity(flywheelVelocitySetpointMPS / ShooterWheelConstants.kCircumferenceM)
            .withAcceleration(acceleration / ShooterWheelConstants.kCircumferenceM));
  }

  public void configMotor(
      int id, boolean invert, PIDController velocity, SimpleMotorFeedforward velocityFeedforward) {
    flywheelMotor = new TalonFX(id);
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = velocity.getP();
    configs.Slot0.kI = velocity.getI();
    configs.Slot0.kD = velocity.getD();

    configs.Slot0.kS = velocityFeedforward.ks;
    configs.Slot0.kV = velocityFeedforward.kv;
    configs.Slot0.kA = velocityFeedforward.ka;

    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.StatorCurrentLimit = 40;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 40;

    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    configs.Feedback.FeedbackRemoteSensorID = id;

    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configs.MotorOutput.Inverted =
        invert ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

    flywheelMotor.getConfigurator().apply(configs);
  }
}
