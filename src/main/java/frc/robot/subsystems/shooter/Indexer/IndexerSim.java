package frc.robot.subsystems.shooter.Indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerSim implements IndexerIO {
  private DCMotorSim indexerMotorLeft =
      new DCMotorSim(
          DCMotor.getNEO(1), IndexerConstants.kGearing, IndexerConstants.kJKGMetersSquared);
  private DCMotorSim indexerMotorRight =
      new DCMotorSim(
          DCMotor.getNEO(1), IndexerConstants.kGearing, IndexerConstants.kJKGMetersSquared);

  public double indexerAppliedVoltsLeft = 0;
  public double indexerAppliedVoltsRight = 0;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerVelocityRPMLeft = indexerMotorLeft.getAngularVelocityRPM();
    inputs.indexerAppliedVoltsLeft = indexerAppliedVoltsLeft;
    inputs.indexerCurrentAmpsLeft = indexerMotorLeft.getCurrentDrawAmps();
    inputs.indexerTemperatureLeftC = 0.0;

    inputs.indexerVelocityRPMRight = indexerMotorRight.getAngularVelocityRPM();
    inputs.indexerAppliedVoltsRight = indexerAppliedVoltsRight;
    inputs.indexerCurrentAmpsRight = indexerMotorRight.getCurrentDrawAmps();
    inputs.indexerTemperatureRightC = 0.0;

    indexerMotorLeft.update(0.02);
    indexerMotorRight.update(0.02);
  }

  @Override
  public void setIndexerVoltsLeft(double volts) {
    indexerAppliedVoltsLeft = volts;
    indexerMotorLeft.setInputVoltage(volts);
  }

  @Override
  public void setIndexerVoltsRight(double volts) {
    indexerAppliedVoltsRight = volts;
    indexerMotorRight.setInputVoltage(volts);
  }
}
