// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterrefactored;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooterrefactored.angler.AnglerIO;
import frc.robot.subsystems.shooterrefactored.angler.AnglerIOInputsAutoLogged;
import frc.robot.subsystems.shooterrefactored.indexer.IndexerIO;
import frc.robot.subsystems.shooterrefactored.indexer.IndexerIOInputsAutoLogged;
import frc.robot.subsystems.shooterrefactored.launcher.LauncherIO;
import frc.robot.subsystems.shooterrefactored.launcher.LauncherIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/** Shooter subsystem */
public class Shooter extends SubsystemBase {
  private AnglerIO anglerIO;
  private AnglerIOInputsAutoLogged anglerIOInputs = new AnglerIOInputsAutoLogged();
  private IndexerIO indexerIO;
  private IndexerIOInputsAutoLogged indexerIOInputs = new IndexerIOInputsAutoLogged();
  private LauncherIO launcherIO;
  private LauncherIOInputsAutoLogged launcherIOInputs = new LauncherIOInputsAutoLogged();

  /** Creates a new Shooter. */
  public Shooter(AnglerIO anglerIO, IndexerIO indexerIO, LauncherIO launcherIO) {
    this.anglerIO = anglerIO;
    this.indexerIO = indexerIO;
    this.launcherIO = launcherIO;
  }

  @Override
  public void periodic() {
    anglerIO.updateInputs(anglerIOInputs);
    Logger.processInputs("Shooter/Angler/Inputs", anglerIOInputs);
    indexerIO.updateInputs(indexerIOInputs);
    Logger.processInputs("Shooter/Indexer/Inputs", indexerIOInputs);
    launcherIO.updateInputs(launcherIOInputs);
    Logger.processInputs("Shooter/Launcher/Inputs", launcherIOInputs);
  }

  public void setLauncherVolts(double topVolts, double bottomVolts) {
    launcherIO.setTopVolts(topVolts);
    launcherIO.setBottomVolts(bottomVolts);
  }

  public void setLauncherVelocity(double topVeloicty, double bottomVelocity) {
    launcherIO.setTopVelocity(topVeloicty);
    launcherIO.setBottomVelocity(bottomVelocity);
  }
}
