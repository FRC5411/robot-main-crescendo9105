package frc.robot.subsystems.shooter.Flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.LoggedTunableNumber;

public class ShooterWheelSimIO implements ShooterWheelIO {
  private FlywheelSim flywheelMotor =
      new FlywheelSim(
          DCMotor.getFalcon500(1),
          ShooterWheelConstants.kGearing,
          ShooterWheelConstants.kJKgMetersSquared,
          VecBuilder.fill(0.1));

  private double appliedVolts = 0;
  private double velocityMeasuredMPS = 0;
  private double velocitySetpointMPS = 0;
  private double velocityRateLimit = 0;

  private PIDController velocityController;
  private SimpleMotorFeedforward velocityFeedforward;
  private SlewRateLimiter velocityRateLimiter;

  private LoggedTunableNumber shooterWheelControlP;
  private LoggedTunableNumber shooterWheelControlI;
  private LoggedTunableNumber shooterWheelControlD;

  public ShooterWheelSimIO(
      PIDController flywheelPID,
      SimpleMotorFeedforward flywheelFeedforward,
      double flywheelRateLimit,
      String key) {
    this.velocityController = flywheelPID;
    this.velocityFeedforward = flywheelFeedforward;
    this.velocityRateLimit = flywheelRateLimit;
    this.velocityRateLimiter = new SlewRateLimiter(this.velocityRateLimit);

    this.shooterWheelControlP =
        new LoggedTunableNumber("ShooterWheel/P" + key, velocityController.getP());
    this.shooterWheelControlI =
        new LoggedTunableNumber("ShooterWheel/I" + key, velocityController.getI());
    this.shooterWheelControlD =
        new LoggedTunableNumber("ShooterWheel/D" + key, velocityController.getD());
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
    appliedVolts = MathUtil.clamp(volts, -12, 12);
    flywheelMotor.setInputVoltage(appliedVolts);
  }

  @Override
  public void setFlywheelsVelocity(double velocityMPS) {
    double velocityMPSLimited = velocityRateLimiter.calculate(velocityMPS);
    velocitySetpointMPS = velocityMPSLimited;
    double PID = velocityController.calculate(velocityMeasuredMPS, velocitySetpointMPS);
    if (velocityMPSLimited - velocityMPS > 1e-1)
      PID +=
          velocityFeedforward.calculate(
              velocitySetpointMPS, ShooterWheelConstants.kFlywheelRateLimit);
    else PID += velocityFeedforward.calculate(velocitySetpointMPS);

    setFlywheelsVolts(PID);
  }
}
