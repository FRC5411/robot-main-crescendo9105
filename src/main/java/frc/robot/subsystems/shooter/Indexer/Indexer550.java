package frc.robot.subsystems.shooter.Indexer;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

public class Indexer550 implements IndexerIO {
  public CANSparkMax indexerLeft;

  public Indexer550() {
    indexerLeft = new CANSparkMax(IndexerConstants.kMotorIDLeft, CANSparkMax.MotorType.kBrushless);

    configIndexMotor(indexerLeft);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerVelocityRPMLeft = indexerLeft.getEncoder().getVelocity();
    inputs.indexerAppliedVoltsLeft = indexerLeft.getAppliedOutput();
    inputs.indexerCurrentAmpsLeft = indexerLeft.getOutputCurrent();
    inputs.indexerTemperatureLeftC = indexerLeft.getMotorTemperature();
  }

  @Override
  public void setIndexerVolts(double volts) {
    indexerLeft.setVoltage(volts);
  }

  private void configIndexMotor(CANSparkMax motor) {
    motor.restoreFactoryDefaults();
    motor.clearFaults();
    motor.setSmartCurrentLimit(20);
    motor.setSecondaryCurrentLimit(20);
    motor.setInverted(false);
    motor.enableVoltageCompensation(12);
    motor.setCANTimeout(10);
    motor.setIdleMode(IdleMode.kBrake);
    motor.burnFlash();
  }
}
