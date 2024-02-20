package frc.robot.subsystems.shooter.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double indexerVelocityRPMLeft = 0.0;
    public double indexerAppliedVoltsLeft = 0.0;
    public double indexerCurrentAmpsLeft = 0.0;
    public double indexerTemperatureLeftC = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setIndexerVolts(double volts) {}
}
