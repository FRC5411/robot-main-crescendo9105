// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.utils.debugging.LoggedTunableNumber;

/** Class to interact with the physical swerve module structure, SDS L2+ */
public class ModuleIOSparkMax implements ModuleIO {
  private final double DRIVE_GEAR_RATIO = 6.75 / 1.0;
  private final double AZIMUTH_GEAR_RATIO = 150.0 / 7.0;
  private final double CIRCUMFRENCE_METERS = 2.0 * Math.PI * (5.08 / 100);

  private CANSparkMax driveMotor;
  private CANSparkMax azimuthMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder azimuthEncoder;
  private CANcoder angleEncoder;

  private Rotation2d angleOffset;

  private SparkPIDController driveFeedback;
  private SparkPIDController azimuthFeedback;

  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 0.27 / 12.0);

  private LoggedTunableNumber driveFeedbackP =
      new LoggedTunableNumber("Drive/ModuleIO/Drive/Feedback/P", 0.0001);
  private LoggedTunableNumber driveFeedbackI =
      new LoggedTunableNumber("Drive/ModuleIO/Drive/Feedback/I", 0.0);
  private LoggedTunableNumber driveFeedbackD =
      new LoggedTunableNumber("Drive/ModuleIO/Drive/Feedback/D", 0.0);

  private LoggedTunableNumber azimuthFeedbackP =
      new LoggedTunableNumber("Drive/ModuleIO/Azimuth/Feedback/P", 0.175);
  private LoggedTunableNumber azimuthFeedbackI =
      new LoggedTunableNumber("Drive/ModuleIO/Azimuth/Feedback/I", 0.0);
  private LoggedTunableNumber azimuthFeedbackD =
      new LoggedTunableNumber("Drive/ModuleIO/Azimuth/Feedback/D", 0.0);

  /** Create a new hardware implementation of a swerve module */
  public ModuleIOSparkMax(int module) {
    switch (module) {
      case 0:
        driveMotor = new CANSparkMax(11, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(21, MotorType.kBrushless);

        angleEncoder = new CANcoder(31, "CTREBUS");
        angleOffset = Rotation2d.fromRotations(-0.275879);

        break;
      case 1:
        driveMotor = new CANSparkMax(12, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(22, MotorType.kBrushless);

        angleEncoder = new CANcoder(32, "CTREBUS");
        angleOffset = Rotation2d.fromRotations(-0.273926);

        break;
      case 2:
        driveMotor = new CANSparkMax(13, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(23, MotorType.kBrushless);

        angleEncoder = new CANcoder(33, "CTREBUS");
        angleOffset = Rotation2d.fromRotations(-0.390137);

        break;
      case 3:
        driveMotor = new CANSparkMax(14, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(24, MotorType.kBrushless);

        angleEncoder = new CANcoder(34, "CTREBUS");
        angleOffset = Rotation2d.fromRotations(0.382568);

        break;
      default:
        throw new RuntimeException("Invalid module");
    }

    driveEncoder = driveMotor.getEncoder();
    azimuthEncoder = azimuthMotor.getEncoder();

    driveFeedback = driveMotor.getPIDController();
    azimuthFeedback = azimuthMotor.getPIDController();

    driveMotor.restoreFactoryDefaults();
    azimuthMotor.restoreFactoryDefaults();

    driveMotor.setCANTimeout(250);
    azimuthMotor.setCANTimeout(250);

    driveMotor.setInverted(false);
    azimuthMotor.setInverted(true);

    driveMotor.setSmartCurrentLimit(40);
    driveMotor.enableVoltageCompensation(12.0);
    azimuthMotor.setSmartCurrentLimit(30);
    azimuthMotor.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    azimuthEncoder.setPosition(0.0);
    resetAzimuthEncoder();
    azimuthEncoder.setMeasurementPeriod(10);
    azimuthEncoder.setAverageDepth(2);

    driveMotor.setIdleMode(IdleMode.kBrake);
    azimuthMotor.setIdleMode(IdleMode.kCoast);

    driveMotor.setCANTimeout(0);
    azimuthMotor.setCANTimeout(0);

    driveFeedback.setP(0.0001);
    driveFeedback.setI(0.0);
    driveFeedback.setD(0.0);
    driveFeedback.setFeedbackDevice(driveEncoder);

    azimuthFeedback.setP(0.175);
    azimuthFeedback.setI(0.0);
    azimuthFeedback.setD(0.0);
    azimuthFeedback.setFeedbackDevice(azimuthEncoder);

    azimuthFeedback.setPositionPIDWrappingEnabled(true);

    // Wrap PID output if output exceeds these boundaries
    // TBH I really don't know why we chose these as the bounds but ¯\_(ツ)_/¯
    azimuthFeedback.setPositionPIDWrappingMinInput(-0.5 * AZIMUTH_GEAR_RATIO);
    azimuthFeedback.setPositionPIDWrappingMaxInput(0.5 * AZIMUTH_GEAR_RATIO);

    driveMotor.burnFlash();
    azimuthMotor.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionM = driveEncoder.getPosition() * CIRCUMFRENCE_METERS / DRIVE_GEAR_RATIO;
    inputs.driveVelocityMPS =
        driveEncoder.getVelocity() * CIRCUMFRENCE_METERS / (60.0 * DRIVE_GEAR_RATIO);
    inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveMotor.getOutputCurrent()};
    inputs.driveTemperatureCelsius = new double[] {driveMotor.getMotorTemperature()};

    inputs.azimuthAbsolutePosition =
        Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble())
            .minus(angleOffset);
    inputs.azimuthPosition =
        Rotation2d.fromRotations(azimuthEncoder.getPosition() / AZIMUTH_GEAR_RATIO);
    inputs.azimuthVelocityRPS =
        Units.rotationsPerMinuteToRadiansPerSecond(azimuthEncoder.getVelocity())
            / AZIMUTH_GEAR_RATIO;
    inputs.azimuthAppliedVolts = azimuthMotor.getAppliedOutput() * azimuthMotor.getBusVoltage();
    inputs.azimuthCurrentAmps = new double[] {azimuthMotor.getOutputCurrent()};
    inputs.azimuthTemperatureCelsius = new double[] {azimuthMotor.getMotorTemperature()};

    if (Constants.tuningMode) {
      updateTunableNumbers();
    }
  }

  @Override
  public void setDriveVolts(double volts) {
    double clampedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveMotor.setVoltage(clampedVolts);
  }

  @Override
  public void setAzimuthVolts(double volts) {
    double clampedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    azimuthMotor.setVoltage(clampedVolts);
  }

  @Override
  public void setDriveVelocity(double velocityMPS) {
    // Adjusting to RPM
    double adjustedVelocity = 60.0 * (velocityMPS / CIRCUMFRENCE_METERS);

    double feedforwardOutput = driveFeedforward.calculate(adjustedVelocity);
    driveFeedback.setReference(adjustedVelocity, ControlType.kVelocity, 0, feedforwardOutput);
  }

  @Override
  public void setAzimuthPosition(Rotation2d position) {
    double feedforwardOutput = 0.0;
    azimuthFeedback.setReference(
        position.getRotations() * AZIMUTH_GEAR_RATIO, ControlType.kPosition, 0, feedforwardOutput);
  }

  @Override
  public void setDriveBrake(boolean isBrake) {
    if (isBrake) {
      driveMotor.setIdleMode(IdleMode.kBrake);
    } else {
      driveMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  @Override
  public void setAzimuthBrake(boolean isBrake) {
    if (isBrake) {
      azimuthMotor.setIdleMode(IdleMode.kBrake);
    } else {
      azimuthMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  /** Update the tunable numbers if they've changed */
  private void updateTunableNumbers() {
    if (driveFeedbackP.hasChanged(hashCode())
        || driveFeedbackI.hasChanged(hashCode())
        || driveFeedbackD.hasChanged(hashCode())) {
      driveFeedback.setP(driveFeedbackP.get());
      driveFeedback.setI(driveFeedbackI.get());
      driveFeedback.setD(driveFeedbackD.get());
    }
    if (azimuthFeedbackP.hasChanged(hashCode())
        || azimuthFeedbackI.hasChanged(hashCode())
        || azimuthFeedbackD.hasChanged(hashCode())) {
      azimuthFeedback.setP(azimuthFeedbackP.get());
      azimuthFeedback.setI(azimuthFeedbackI.get());
      azimuthFeedback.setD(azimuthFeedbackD.get());
    }
  }

  /** Reset the relative azimuth encoder to the absolute position */
  private void resetAzimuthEncoder() {
    Rotation2d currentAbsolutePosition =
        Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble())
            .minus(angleOffset);

    // The number of rotations a motor makes is affected by the gear ratio
    azimuthEncoder.setPosition(currentAbsolutePosition.getRotations() * AZIMUTH_GEAR_RATIO);
  }

  @Override
  public void reset() {
    resetAzimuthEncoder();
  }
}
