// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

/** An SDS MK4i L3 swerve module */
public class ModuleIOSparkMax implements ModuleIO {
  private final boolean IS_AZIMUTH_INVERTED = true;
  // TODO Update these for our latest swerve modules
  private final double DRIVE_GEAR_RATIO = 6.75 / 1.0;
  private final double AZIMUTH_GEAR_RATIO = 150.0 / 7.0;

  private CANSparkMax driveMotor;
  private CANSparkMax azimuthMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder azimuthEncoder;
  private CANcoder angleEncoder;

  private Rotation2d angleOffset;

  private SparkPIDController driveController;
  private SparkPIDController azimuthController;

  public ModuleIOSparkMax(int module) {
    // TODO Update devices and offsets as needed
    switch (module) {
      case 0:
        driveMotor = new CANSparkMax(11, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(21, MotorType.kBrushless);

        angleEncoder = new CANcoder(31);
        angleOffset = new Rotation2d(0.0);

        break;
      case 1:
        driveMotor = new CANSparkMax(12, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(22, MotorType.kBrushless);

        angleEncoder = new CANcoder(32);
        angleOffset = new Rotation2d(0.0);

        break;
      case 2:
        driveMotor = new CANSparkMax(13, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(23, MotorType.kBrushless);

        angleEncoder = new CANcoder(33);
        angleOffset = new Rotation2d(0.0);

        break;
      case 3:
        driveMotor = new CANSparkMax(14, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(24, MotorType.kBrushless);

        angleEncoder = new CANcoder(34);
        angleOffset = new Rotation2d(0.0);

        break;
      default:
        throw new RuntimeException("Invalid module");
    }

    driveEncoder = driveMotor.getEncoder();
    azimuthEncoder = azimuthMotor.getEncoder();

    driveMotor.restoreFactoryDefaults();
    azimuthMotor.restoreFactoryDefaults();

    driveMotor.setCANTimeout(250);
    azimuthMotor.setCANTimeout(250);

    driveMotor.setInverted(false);
    azimuthMotor.setInverted(IS_AZIMUTH_INVERTED);

    driveMotor.setSmartCurrentLimit(40);
    driveMotor.enableVoltageCompensation(12.0);
    azimuthMotor.setSmartCurrentLimit(30);
    azimuthMotor.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    azimuthEncoder.setPosition(0.0);
    azimuthEncoder.setMeasurementPeriod(10);
    azimuthEncoder.setAverageDepth(2);

    // TODO Investigate why this is set to 0 after being set to 250
    driveMotor.setCANTimeout(0);
    azimuthMotor.setCANTimeout(0);

    // TODO Add threaded odometry stuff here

    driveController = driveMotor.getPIDController();
    azimuthController = azimuthMotor.getPIDController();

    // TODO Tune PID gains
    driveController.setP(0.0);
    driveController.setI(0.0);
    driveController.setD(0.0);
    driveController.setFF(0.0);

    azimuthController.setP(0.0);
    azimuthController.setI(0.0);
    azimuthController.setD(0.0);
    azimuthController.setFF(0.0);

    driveMotor.burnFlash();
    azimuthMotor.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionR = Units.rotationsToRadians(driveEncoder.getPosition()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRPS =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveMotor.getOutputCurrent()};

    // TODO Verify that this works lmao
    inputs.azimuthAbsolutePosition =
        new Rotation2d(
                angleEncoder.getSupplyVoltage().getValueAsDouble()
                    / RobotController.getVoltage5V()
                    * 2.0
                    * Math.PI)
            .minus(angleOffset);
    inputs.azimuthPosition =
        Rotation2d.fromRotations(azimuthEncoder.getPosition() / AZIMUTH_GEAR_RATIO);
    inputs.azimuthVelocityRPS =
        Units.rotationsPerMinuteToRadiansPerSecond(azimuthEncoder.getVelocity())
            / AZIMUTH_GEAR_RATIO;
    inputs.azimuthAppliedVolts = azimuthMotor.getAppliedOutput() * azimuthMotor.getBusVoltage();
    inputs.azimuthCurrentAmps = new double[] {azimuthMotor.getOutputCurrent()};

    // TODO Add threaded odometry values
  }

  @Override
  public void setDriveVolts(double volts) {}

  @Override
  public void setAzimuthVolts(double volts) {}

  @Override
  public void setVelocity(double velocityMetersPerSecond) {}

  @Override
  public void setAngle(double angleRad) {}

  @Override
  public void setDriveBrake(boolean isBrake) {}

  @Override
  public void setAzimuthBrake(boolean isBrake) {}
}
