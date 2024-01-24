package frc.robot.subsystems.shooter.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public double indexerVelocityRPMLeft = 0.0;
        public double indexerAppliedVoltsLeft = 0.0;
        public double indexerCurrentAmpsLeft = 0.0;
        public double indexerTemperatureLeftC = 0.0;

        public double indexerVelocityRPMRight = 0.0;
        public double indexerAppliedVoltsRight = 0.0;
        public double indexerCurrentAmpsRight = 0.0;
        public double indexerTemperatureRightC = 0.0;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void setIndexerVoltsLeft(double volts) {}

    public default void setIndexerVoltsRight(double volts) {}

    public default void setIndexerVolts(double volts) {
        setIndexerVoltsLeft(volts);
        setIndexerVoltsRight(volts);
    }
}
