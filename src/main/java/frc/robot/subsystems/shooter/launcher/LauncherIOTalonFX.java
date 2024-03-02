// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.launcher;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.utils.debugging.LoggedTunableNumber;

/** Class to interact with the physical launcher structure */
public class LauncherIOTalonFX implements LauncherIO {
  private final double GEARING = 1.0 / 1.0;
  private final double RADIUS_M = 6.35 / 100;
  private final double CIRCUMFRENCE_M = 2.0 * Math.PI * RADIUS_M;
  private final double RATE_LIMIT = 25.0;

  private TalonFX topMotor = new TalonFX(43);
  private TalonFX bottomMotor = new TalonFX(44);

  private TalonFXConfiguration topConfiguration = new TalonFXConfiguration();
  private TalonFXConfiguration bottomConfiguration = new TalonFXConfiguration();

  private VelocityVoltage topVelocityVoltage = new VelocityVoltage(0.0);
  private VelocityVoltage bottomVelocityVoltage = new VelocityVoltage(0.0);

  private SlewRateLimiter topLimiter = new SlewRateLimiter(RATE_LIMIT);
  private SlewRateLimiter bottomLimiter = new SlewRateLimiter(RATE_LIMIT);

  private LoggedTunableNumber topFeedbackP =
      new LoggedTunableNumber("Shooter/LauncherTop/Feedback/P", 0.040193);
  private LoggedTunableNumber topFeedbackI =
      new LoggedTunableNumber("Shooter/LauncherTop/Feedback/I", 0.0);
  private LoggedTunableNumber topFeedbackD =
      new LoggedTunableNumber("Shooter/LauncherTop/Feedback/D", 0.0);

  private LoggedTunableNumber bottomFeedbackP =
      new LoggedTunableNumber("Shooter/LauncherBottom/Feedback/P", 0.040193);
  private LoggedTunableNumber bottomFeedbackI =
      new LoggedTunableNumber("Shooter/LauncherBottom/Feedback/I", 0.0);
  private LoggedTunableNumber bottomFeedbackD =
      new LoggedTunableNumber("Shooter/LauncherBottom/Feedback/D", 0.0);

  private LoggedTunableNumber topFeedbackS =
      new LoggedTunableNumber("Shooter/LauncherTop/Feedback/S", 0.003384);
  private LoggedTunableNumber topFeedbackG =
      new LoggedTunableNumber("Shooter/LauncherTop/Feedback/G", 0.0);
  private LoggedTunableNumber topFeedbackV =
      new LoggedTunableNumber("Shooter/LauncherTop/Feedback/V", 0.11036);
  private LoggedTunableNumber topFeedbackA =
      new LoggedTunableNumber("Shooter/LauncherTop/Feedback/A", 0.0);

  private LoggedTunableNumber bottomFeedbackS =
      new LoggedTunableNumber("Shooter/LauncherBottom/Feedback/S", 0.003384);
  private LoggedTunableNumber bottomFeedbackG =
      new LoggedTunableNumber("Shooter/LauncherBottom/Feedback/G", 0.0);
  private LoggedTunableNumber bottomFeedbackV =
      new LoggedTunableNumber("Shooter/LauncherBottom/Feedback/V", 0.11036);
  private LoggedTunableNumber bottomFeedbackA =
      new LoggedTunableNumber("Shooter/LauncherBottom/Feedback/A", 0.0);

  private LoggedTunableNumber topLimiterValue =
      new LoggedTunableNumber("Shooter/LauncherTop/Limiter/Value", RATE_LIMIT);
  private LoggedTunableNumber bottomLimiterValue =
      new LoggedTunableNumber("Shooter/LauncherBottom/Limiter/Value", RATE_LIMIT);

  private double topAppliedVolts = 0.0;
  private double bottomAppliedVolts = 0.0;

  private double topVelocityMPS = 0.0;
  private double bottomVelocityMPS = 0.0;

  /** Create a new hardware implementation of the launcher */
  public LauncherIOTalonFX() {
    topConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    topConfiguration.CurrentLimits.StatorCurrentLimit = 40;
    bottomConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    bottomConfiguration.CurrentLimits.StatorCurrentLimit = 40;
    topConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    topConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
    bottomConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    bottomConfiguration.CurrentLimits.SupplyCurrentLimit = 40;

    topConfiguration.Voltage.PeakForwardVoltage = 12.0;
    topConfiguration.Voltage.PeakReverseVoltage = -12.0;
    bottomConfiguration.Voltage.PeakForwardVoltage = 12.0;
    bottomConfiguration.Voltage.PeakReverseVoltage = -12.0;

    topConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    bottomConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    topConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    bottomConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    topConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    bottomConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    topConfiguration.Slot0.kP = topFeedbackP.get();
    topConfiguration.Slot0.kI = topFeedbackI.get();
    topConfiguration.Slot0.kD = topFeedbackD.get();

    bottomConfiguration.Slot0.kP = bottomFeedbackP.get();
    bottomConfiguration.Slot0.kI = bottomFeedbackI.get();
    bottomConfiguration.Slot0.kD = bottomFeedbackD.get();

    topConfiguration.Slot0.kS = topFeedbackS.get();
    topConfiguration.Slot0.kG = topFeedbackG.get();
    topConfiguration.Slot0.kV = topFeedbackV.get();
    topConfiguration.Slot0.kA = topFeedbackA.get();

    bottomConfiguration.Slot0.kS = bottomFeedbackS.get();
    bottomConfiguration.Slot0.kG = bottomFeedbackG.get();
    bottomConfiguration.Slot0.kV = bottomFeedbackV.get();
    bottomConfiguration.Slot0.kA = bottomFeedbackA.get();

    topMotor.getConfigurator().apply(topConfiguration);
    bottomMotor.getConfigurator().apply(bottomConfiguration);
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    inputs.topFlywheelVelocityMPS =
        (topMotor.getVelocity().getValueAsDouble() * CIRCUMFRENCE_M) / GEARING;
    inputs.topFlywheelAppliedVolts = topAppliedVolts;
    inputs.topFlywheelInternalVolts = topMotor.getMotorVoltage().getValueAsDouble();
    inputs.topFlywheelAppliedCurrentAmps =
        new double[] {topMotor.getStatorCurrent().getValueAsDouble()};
    inputs.topFlywheelTemperatureCelsius =
        new double[] {topMotor.getDeviceTemp().getValueAsDouble()};

    inputs.bottomFlywheelVelocityMPS =
        (bottomMotor.getVelocity().getValueAsDouble() * CIRCUMFRENCE_M) / GEARING;
    inputs.bottomFlywheelAppliedVolts = bottomAppliedVolts;
    inputs.bottomFlywheelInternalVolts = bottomMotor.getMotorVoltage().getValueAsDouble();
    inputs.bottomFlywheelAppliedCurrentAmps =
        new double[] {bottomMotor.getStatorCurrent().getValueAsDouble()};
    inputs.bottomFlywheelTemperatureCelsius =
        new double[] {bottomMotor.getDeviceTemp().getValueAsDouble()};

    updateTunableNumbers();
  }

  @Override
  public void setTopVolts(double volts) {
    topAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    topMotor.setVoltage(topAppliedVolts);
  }

  @Override
  public void setBottomVolts(double volts) {
    bottomAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    bottomMotor.setVoltage(bottomAppliedVolts);
  }

  @Override
  public void setTopVelocity(double velocityMPS) {
    topVelocityMPS = topLimiter.calculate(velocityMPS);
    double accelerationMPS = 0.0;

    if (topVelocityMPS - velocityMPS > 1e-1) {
      accelerationMPS = RATE_LIMIT / CIRCUMFRENCE_M;
    }

    topMotor.setControl(
        topVelocityVoltage
            .withVelocity(topVelocityMPS / CIRCUMFRENCE_M)
            .withAcceleration(accelerationMPS));
  }

  @Override
  public void setBottomVelocity(double velocityMPS) {
    bottomVelocityMPS = bottomLimiter.calculate(velocityMPS);
    double accelerationMPS = 0.0;

    if (bottomVelocityMPS - velocityMPS > 1e-1) {
      accelerationMPS = RATE_LIMIT / CIRCUMFRENCE_M;
    }

    bottomMotor.setControl(
        bottomVelocityVoltage
            .withVelocity(bottomVelocityMPS / CIRCUMFRENCE_M)
            .withAcceleration(accelerationMPS));
  }

  /** Update the tunable numbers if they've changed */
  private void updateTunableNumbers() {
    if (topFeedbackP.hasChanged(hashCode())
        || topFeedbackI.hasChanged(hashCode())
        || topFeedbackD.hasChanged(hashCode())) {

      topConfiguration.Slot0.kP = topFeedbackP.get();
      topConfiguration.Slot0.kI = topFeedbackI.get();
      topConfiguration.Slot0.kD = topFeedbackD.get();

      topMotor.getConfigurator().apply(topConfiguration);
    }
    if (topFeedbackS.hasChanged(hashCode())
        || topFeedbackG.hasChanged(hashCode())
        || topFeedbackV.hasChanged(hashCode())
        || topFeedbackA.hasChanged(hashCode())) {
      topConfiguration.Slot0.kS = topFeedbackS.get();
      topConfiguration.Slot0.kG = topFeedbackG.get();
      topConfiguration.Slot0.kV = topFeedbackV.get();
      topConfiguration.Slot0.kA = topFeedbackA.get();

      topMotor.getConfigurator().apply(topConfiguration);
    }
    if (bottomFeedbackP.hasChanged(hashCode())
        || bottomFeedbackI.hasChanged(hashCode())
        || bottomFeedbackD.hasChanged(hashCode())) {

      bottomConfiguration.Slot0.kP = bottomFeedbackP.get();
      bottomConfiguration.Slot0.kI = bottomFeedbackI.get();
      bottomConfiguration.Slot0.kD = bottomFeedbackD.get();

      bottomMotor.getConfigurator().apply(bottomConfiguration);
    }
    if (bottomFeedbackS.hasChanged(hashCode())
        || bottomFeedbackG.hasChanged(hashCode())
        || bottomFeedbackV.hasChanged(hashCode())
        || bottomFeedbackA.hasChanged(hashCode())) {
      bottomConfiguration.Slot0.kS = bottomFeedbackS.get();
      bottomConfiguration.Slot0.kG = bottomFeedbackG.get();
      bottomConfiguration.Slot0.kV = bottomFeedbackV.get();
      bottomConfiguration.Slot0.kA = bottomFeedbackA.get();

      bottomMotor.getConfigurator().apply(bottomConfiguration);
    }
    if (topLimiterValue.hasChanged(hashCode()) || bottomLimiterValue.hasChanged(hashCode())) {
      topLimiter = new SlewRateLimiter(topLimiterValue.get());
      bottomLimiter = new SlewRateLimiter(bottomLimiterValue.get());
    }
  }
}
