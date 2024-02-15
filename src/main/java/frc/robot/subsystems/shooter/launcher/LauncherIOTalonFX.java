// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.launcher;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import frc.robot.utils.debugging.LoggedTunableNumber;

/** Class to interact with the physical launcher */
public class LauncherIOTalonFX implements LauncherIO {
  private final double RADIUS_M = 7.62 / 100.0;
  private final double CIRCUMFRENCE_M = 2.0 * Math.PI * RADIUS_M;

  private VelocityVoltage flywheelVelocity = new VelocityVoltage(0.0);

  // TODO Update as needed
  private TalonFX topMotor = new TalonFX(43);
  private TalonFX bottomMotor = new TalonFX(44);

  private double topAppliedVolts = 0.0;
  private double bottomAppliedVolts = 0.0;

  private LoggedTunableNumber topFeedbackP = new LoggedTunableNumber("Shooter/LauncherIO/TopRoller/Feedback/P", 0.0);
  private LoggedTunableNumber topFeedbackI = new LoggedTunableNumber("Shooter/LauncherIO/TopRoller/Feedback/I", 0.0);
  private LoggedTunableNumber topFeedbackD = new LoggedTunableNumber("Shooter/LauncherIO/TopRoller/Feedback/D", 0.0);

  private LoggedTunableNumber bottomFeedbackP =
      new LoggedTunableNumber("Shooter/LauncherIO/BottomRoller/Feedback/P", 0.0);
  private LoggedTunableNumber bottomFeedbackI =
      new LoggedTunableNumber("Shooter/LauncherIO/BottomRoller/Feedback/I", 0.0);
  private LoggedTunableNumber bottomFeedbackD =
      new LoggedTunableNumber("Shooter/LauncherIO/BottomRoller/Feedbac/D", 0.0);

  private LoggedTunableNumber topFeedbackS = new LoggedTunableNumber("Shooter/LauncherIO/TopRoller/Feedback/S", 0.0);
  private LoggedTunableNumber topFeedbackG = new LoggedTunableNumber("Shooter/LauncherIO/TopRoller/Feedback/G", 0.0);
  private LoggedTunableNumber topFeedbackV = new LoggedTunableNumber("Shooter/LauncherIO/TopRoller/Feedback/V", 0.0);
  private LoggedTunableNumber topFeedbackA = new LoggedTunableNumber("Shooter/LauncherIO/TopRoller/Feedback/A", 0.0);

  private LoggedTunableNumber bottomFeedbackS =
      new LoggedTunableNumber("Shooter/LauncherIO/BottomRoller/Feedbac/kS", 0.0);
  private LoggedTunableNumber bottomFeedbackG =
      new LoggedTunableNumber("Shooter/LauncherIO/BottomRoller/Feedback/G", 0.0);
  private LoggedTunableNumber bottomFeedbackV =
      new LoggedTunableNumber("Shooter/LauncherIO/BottomRoller/Feedback/V", 0.0);
  private LoggedTunableNumber bottomFeedbackA =
      new LoggedTunableNumber("Shooter/LauncherIO/BottomRoller/Feedback/A", 0.0);

  /** Create a new hardware implementation of the launcher */
  public LauncherIOTalonFX() {
    // TODO Update as needed
    TalonFXConfiguration topConfiguration = new TalonFXConfiguration();
    TalonFXConfiguration bottomConfiguration = new TalonFXConfiguration();

    topConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    topConfiguration.CurrentLimits.StatorCurrentLimit = 40;
    bottomConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    bottomConfiguration.CurrentLimits.StatorCurrentLimit = 40;

    topConfiguration.Voltage.PeakForwardVoltage = 12.0;
    topConfiguration.Voltage.PeakReverseVoltage = -12.0;
    bottomConfiguration.Voltage.PeakForwardVoltage = 12.0;
    bottomConfiguration.Voltage.PeakReverseVoltage = -12.0;

    topConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    bottomConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    topMotor.setInverted(true);
    bottomMotor.setInverted(false);

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
    // TODO Update as needed
    inputs.topAngleRadians = topMotor.getPosition().getValueAsDouble();
    inputs.topVelocityRPM = topMotor.getVelocity().getValueAsDouble();
    inputs.topAppliedVolts = topAppliedVolts;
    inputs.topAppliedCurrentAmps = new double[] {topMotor.getStatorCurrent().getValueAsDouble()};
    inputs.topTemperatureCelsius = new double[] {topMotor.getDeviceTemp().getValueAsDouble()};

    inputs.bottomAngleRadians = bottomMotor.getPosition().getValueAsDouble();
    inputs.bottomVelocityRPM = bottomMotor.getVelocity().getValueAsDouble();
    inputs.bottomAppliedVolts = bottomAppliedVolts;
    inputs.bottomAppliedCurrentAmps =
        new double[] {bottomMotor.getStatorCurrent().getValueAsDouble()};
    inputs.bottomTemperatureCelsius = new double[] {bottomMotor.getDeviceTemp().getValueAsDouble()};

    updateTunableNumbers();
  }

  @Override
  public void setVolts(double topVolts, double bottomVolts) {
    topAppliedVolts = MathUtil.clamp(topVolts, -12.0, 12.0);
    bottomAppliedVolts = MathUtil.clamp(bottomVolts, -12.0, 12.0);

    topMotor.setVoltage(topAppliedVolts);
    bottomMotor.setVoltage(bottomAppliedVolts);
  }

  @Override
  public void setVelocity(double velocityMPS) {
    topMotor.setControl(flywheelVelocity.withVelocity(velocityMPS / CIRCUMFRENCE_M));
    bottomMotor.setControl(flywheelVelocity.withVelocity(velocityMPS / CIRCUMFRENCE_M));
  }

  /** Update the tunable numbers if they've changed */
  private void updateTunableNumbers() {
    if (topFeedbackP.hasChanged(hashCode())
        || topFeedbackI.hasChanged(hashCode())
        || topFeedbackD.hasChanged(hashCode())) {
      TalonFXConfiguration topConfiguration = new TalonFXConfiguration();

      topConfiguration.Slot0.kP = topFeedbackP.get();
      topConfiguration.Slot0.kI = topFeedbackI.get();
      topConfiguration.Slot0.kD = topFeedbackD.get();

      if (topFeedbackS.hasChanged(hashCode())
          || topFeedbackG.hasChanged(hashCode())
          || topFeedbackV.hasChanged(hashCode())
          || topFeedbackA.hasChanged(hashCode())) {
        topConfiguration.Slot0.kS = topFeedbackS.get();
        topConfiguration.Slot0.kG = topFeedbackG.get();
        topConfiguration.Slot0.kV = topFeedbackV.get();
        topConfiguration.Slot0.kA = topFeedbackA.get();
      }

      topMotor.getConfigurator().apply(topConfiguration);
    }
    if (bottomFeedbackP.hasChanged(hashCode())
        || bottomFeedbackI.hasChanged(hashCode())
        || bottomFeedbackD.hasChanged(hashCode())) {
      TalonFXConfiguration bottomConfiguration = new TalonFXConfiguration();

      bottomConfiguration.Slot0.kP = bottomFeedbackP.get();
      bottomConfiguration.Slot0.kI = bottomFeedbackI.get();
      bottomConfiguration.Slot0.kD = bottomFeedbackD.get();

      if (bottomFeedbackS.hasChanged(hashCode())
          || bottomFeedbackG.hasChanged(hashCode())
          || bottomFeedbackV.hasChanged(hashCode())
          || bottomFeedbackA.hasChanged(hashCode())) {
        bottomConfiguration.Slot0.kS = bottomFeedbackS.get();
        bottomConfiguration.Slot0.kG = bottomFeedbackG.get();
        bottomConfiguration.Slot0.kV = bottomFeedbackV.get();
        bottomConfiguration.Slot0.kA = bottomFeedbackA.get();
      }

      bottomMotor.getConfigurator().apply(bottomConfiguration);
    }
  }
}
