// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private IndexerIO indexerIO;
  private IndexerIOInputsAutoLogged indexerIOInputs = new IndexerIOInputsAutoLogged();

  // TODO Update source
  private DigitalInput beamBreakSensor = new DigitalInput(4);

  /** Creates a new Indexer. */
  public Indexer(IndexerIO indexerIO) {
    this.indexerIO = indexerIO;
  }

  @Override
  public void periodic() {
    indexerIO.updateInputs(indexerIOInputs);
    Logger.processInputs("Indexer/Inputs", indexerIOInputs);
  }

  /** Stop the indexer motor */
  public void stopMotors() {
    indexerIO.setVolts(0.0);
  }

  /** Set the voltage of the indexer motor */
  public void setIndexerVolts(double volts) {
    indexerIO.setVolts(volts);
  }

  /** Returns the status of the beam break sensor; checks if a note is stowed or not */
  public boolean isBeamBroken() {
    return beamBreakSensor.get();
  }
}
