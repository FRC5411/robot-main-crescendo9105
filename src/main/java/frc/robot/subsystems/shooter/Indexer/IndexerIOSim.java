package frc.robot.subsystems.shooter.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
  private DCMotorSim indexerMotorLeft =
      new DCMotorSim(
          DCMotor.getNEO(1), IndexerConstants.kGearing, IndexerConstants.kJKGMetersSquared);

  public double indexerAppliedVoltsLeft = 0;
  public double indexerAppliedVoltsRight = 0;

  @Override
  public void updateInputs(IndexerIOInputsI inputs) {
    inputs.indexerVelocityRPMLeft = indexerMotorLeft.getAngularVelocityRPM();
    inputs.indexerAppliedVoltsLeft = indexerAppliedVoltsLeft;
    inputs.indexerCurrentAmpsLeft = indexerMotorLeft.getCurrentDrawAmps();
    inputs.indexerTemperatureLeftC = 0.0;

    indexerMotorLeft.update(0.02);
  }

  @Override
  public void setIndexerVolts(double volts) {
    indexerAppliedVoltsLeft = volts;
    indexerMotorLeft.setInputVoltage(volts);
  }
}
