// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.launcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.utils.debugging.LoggedTunableNumber;

/** Class to interact with the physical launcher structure */
public class LauncherIOTalonFX implements LauncherIO {
  private final double GEARING = 1.0 / 1.0;
  private final double RADIUS_M = 6.35 / 100;
  private final double CIRCUMFRENCE_M = 2.0 * Math.PI * RADIUS_M;

  private TalonFX topMotor = new TalonFX(43);
  private TalonFX bottomMotor = new TalonFX(44);

  private TalonFXConfiguration topConfiguration = new TalonFXConfiguration();
  private TalonFXConfiguration bottomConfiguration = new TalonFXConfiguration();

  private VelocityVoltage topVelocityVoltage = new VelocityVoltage(0.0);
  private VelocityVoltage bottomVelocityVoltage = new VelocityVoltage(0.0);

  private LoggedTunableNumber topFeedbackP =
      new LoggedTunableNumber("Shooter/LauncherTop/Feedback/P", 0.03);
  private LoggedTunableNumber topFeedbackI =
      new LoggedTunableNumber("Shooter/LauncherTop/Feedback/I", 0.0);
  private LoggedTunableNumber topFeedbackD =
      new LoggedTunableNumber("Shooter/LauncherTop/Feedback/D", 0.0);

  private LoggedTunableNumber bottomFeedbackP =
      new LoggedTunableNumber("Shooter/LauncherBottom/Feedback/P", 0.03257);
  private LoggedTunableNumber bottomFeedbackI =
      new LoggedTunableNumber("Shooter/LauncherBottom/Feedback/I", 0.0);
  private LoggedTunableNumber bottomFeedbackD =
      new LoggedTunableNumber("Shooter/LauncherBottom/Feedback/D", 0.0);

  private LoggedTunableNumber topFeedforwardS =
      new LoggedTunableNumber("Shooter/LauncherTop/Feedforward/S", 0.0060808);
  private LoggedTunableNumber topFeedforwardV =
      new LoggedTunableNumber("Shooter/LauncherTop/Feedforward/V", 0.1105);
  private LoggedTunableNumber topFeedforwardA =
      new LoggedTunableNumber("Shooter/LauncherTop/Feedforward/A", 0.014977);

  private LoggedTunableNumber bottomFeedforwardS =
      new LoggedTunableNumber("Shooter/LauncherBottom/Feedforward/S", 0.060808);
  private LoggedTunableNumber bottomFeedforwardV =
      new LoggedTunableNumber("Shooter/LauncherBottom/Feedforward/V", 0.10994);
  private LoggedTunableNumber bottomFeedforwardA =
      new LoggedTunableNumber("Shooter/LauncherBottom/Feedforward/A", 0.014977);

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

    topConfiguration.Slot0.kS = topFeedforwardS.get();
    topConfiguration.Slot0.kG = 0.0;
    topConfiguration.Slot0.kV = topFeedforwardV.get();
    topConfiguration.Slot0.kA = topFeedforwardA.get();

    bottomConfiguration.Slot0.kS = bottomFeedforwardS.get();
    bottomConfiguration.Slot0.kG = 0.0;
    bottomConfiguration.Slot0.kV = bottomFeedforwardV.get();
    bottomConfiguration.Slot0.kA = bottomFeedforwardA.get();

    topMotor.getConfigurator().apply(topConfiguration);
    bottomMotor.getConfigurator().apply(bottomConfiguration);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        topMotor.getVelocity(),
        topMotor.getMotorVoltage(),
        topMotor.getSupplyCurrent(),
        topMotor.getDeviceTemp(),
        bottomMotor.getVelocity(),
        bottomMotor.getMotorVoltage(),
        bottomMotor.getSupplyCurrent(),
        bottomMotor.getDeviceTemp());

    topMotor.optimizeBusUtilization();
    bottomMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    inputs.topFlywheelVelocityMPS =
        (topMotor.getVelocity().getValueAsDouble() * CIRCUMFRENCE_M) / GEARING;
    inputs.topFlywheelAppliedVolts = topAppliedVolts;
    inputs.topFlywheelInternalVolts = topMotor.getMotorVoltage().getValueAsDouble();
    inputs.topFlywheelAppliedCurrentAmps =
        new double[] {topMotor.getSupplyCurrent().getValueAsDouble()};
    inputs.topFlywheelTemperatureCelsius =
        new double[] {topMotor.getDeviceTemp().getValueAsDouble()};
    inputs.topFlywheelSetpointMPS = topVelocityMPS;
    inputs.topFlywheelErrorMPS = inputs.topFlywheelSetpointMPS - inputs.topFlywheelVelocityMPS;

    inputs.bottomFlywheelVelocityMPS =
        (bottomMotor.getVelocity().getValueAsDouble() * CIRCUMFRENCE_M) / GEARING;
    inputs.bottomFlywheelAppliedVolts = bottomAppliedVolts;
    inputs.bottomFlywheelInternalVolts = bottomMotor.getMotorVoltage().getValueAsDouble();
    inputs.bottomFlywheelAppliedCurrentAmps =
        new double[] {bottomMotor.getSupplyCurrent().getValueAsDouble()};
    inputs.bottomFlywheelTemperatureCelsius =
        new double[] {bottomMotor.getDeviceTemp().getValueAsDouble()};
    inputs.bottomFlywheelSetpointMPS = bottomVelocityMPS;
    inputs.bottomFlywheelErrorMPS = inputs.topFlywheelSetpointMPS - inputs.topFlywheelVelocityMPS;

    if (Constants.tuningMode) {
      updateTunableNumbers();
    }
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
  public void setTopVelocity(double velocityMPS, double accelerationMPS) {
    topVelocityMPS = velocityMPS;
    topMotor.setControl(topVelocityVoltage.withVelocity(topVelocityMPS / CIRCUMFRENCE_M));
    // .withAcceleration(accelerationMPS / CIRCUMFRENCE_M));
  }

  @Override
  public void setBottomVelocity(double velocityMPS, double accelerationMPS) {
    bottomVelocityMPS = velocityMPS;
    bottomMotor.setControl(bottomVelocityVoltage.withVelocity(bottomVelocityMPS / CIRCUMFRENCE_M));
    // .withAcceleration(accelerationMPS / CIRCUMFRENCE_M));
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
    if (topFeedforwardS.hasChanged(hashCode())
        || topFeedforwardV.hasChanged(hashCode())
        || topFeedforwardA.hasChanged(hashCode())) {
      topConfiguration.Slot0.kS = topFeedforwardS.get();
      topConfiguration.Slot0.kV = topFeedforwardV.get();
      topConfiguration.Slot0.kA = topFeedforwardA.get();

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
    if (bottomFeedforwardS.hasChanged(hashCode())
        || bottomFeedforwardV.hasChanged(hashCode())
        || bottomFeedforwardA.hasChanged(hashCode())) {
      bottomConfiguration.Slot0.kS = bottomFeedforwardS.get();
      bottomConfiguration.Slot0.kV = bottomFeedforwardV.get();
      bottomConfiguration.Slot0.kA = bottomFeedforwardA.get();

      bottomMotor.getConfigurator().apply(bottomConfiguration);
    }
  }
}
