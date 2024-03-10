// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IndexerCommands;
import frc.robot.commands.IndexerCommands.IndexerDirection;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.IntakeCommands.IntakeDirection;
import frc.robot.commands.YoshiCommands;
import frc.robot.commands.YoshiCommands.YoshiFlywheelDirection;
import frc.robot.commands.YoshiCommands.YoshiPivotSetpoint;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.yoshivator.Yoshivator;
import org.littletonrobotics.junction.Logger;

/** State-machine for the Intake, Indexer, & Yoshivator */
public class Caster extends SubsystemBase {
  private Intake robotIntake;
  private Indexer robotIndexer;
  private Yoshivator robotYoshi;

  private CasterState currentState = CasterState.IDLE;

  /** Creates a new Caster. */
  public Caster(Intake intake, Indexer indexer, Yoshivator yoshi) {
    robotIntake = intake;
    robotIndexer = indexer;
    robotYoshi = yoshi;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Caster/CurrentState", currentState);
  }

  /** Returns a command based on the current robot state */
  public Command getCommand(CasterState state) {
    return Commands.runOnce(
            () -> {
              currentState = state;
            },
            this)
        .andThen(
            switch (state) {
              case IDLE -> IntakeCommands.stopIntake(robotIntake)
                  .alongWith(IndexerCommands.stopIndexer(robotIndexer))
                  .alongWith(
                      YoshiCommands.runPivotSetpoint(robotYoshi, YoshiPivotSetpoint.IDLE)
                          .andThen(
                              YoshiCommands.runFlywheelManual(
                                  robotYoshi, YoshiFlywheelDirection.STOP)));
              case INTAKE_GROUND -> IntakeCommands.runIntake(robotIntake, IntakeDirection.IN)
                  .alongWith(IndexerCommands.stowPiece(robotIndexer))
                  .alongWith(
                      YoshiCommands.runIntake(
                          robotYoshi,
                          YoshiPivotSetpoint.GROUND,
                          YoshiFlywheelDirection.IN,
                          () -> robotIndexer.isBeamBroken()));
              case OUTTAKE -> IntakeCommands.runIntake(robotIntake, IntakeDirection.OUT)
                  .alongWith(IndexerCommands.stowPiece(robotIndexer))
                  .alongWith(
                      YoshiCommands.runIntake(
                          robotYoshi, YoshiPivotSetpoint.IDLE, YoshiFlywheelDirection.OUT));
              case INDEX -> IndexerCommands.runIndexer(robotIndexer, IndexerDirection.IN)
                  .alongWith(YoshiCommands.runPivotSetpoint(robotYoshi, YoshiPivotSetpoint.IDLE));
              case OUTDEX -> IndexerCommands.runIndexer(robotIndexer, IndexerDirection.OUT)
                  .alongWith(YoshiCommands.runPivotSetpoint(robotYoshi, YoshiPivotSetpoint.IDLE));
              case FIRE -> IntakeCommands.runIntake(robotIntake, IntakeDirection.IN)
                  .alongWith(IndexerCommands.stowPiece(robotIndexer))
                  .alongWith(YoshiCommands.runPivotSetpoint(robotYoshi, YoshiPivotSetpoint.IDLE));
              case INTAKE_AMP -> YoshiCommands.runIntake(
                  robotYoshi, YoshiPivotSetpoint.GROUND, YoshiFlywheelDirection.OUT);
              case SCORE_AMP -> YoshiCommands.scoreAmp(robotYoshi);
            });
  }

  /** Predefined states of the caster */
  public static enum CasterState {
    /** All motors are idle (stopped) */
    IDLE,
    /**
     * Intake is running into the robot - Indexer is running into the robot until a piece is stowed
     * in the shooter - Yoshi is at the ground setpoint spinning into the robot
     */
    INTAKE_GROUND,
    /** Intake is running out of the robot - Indexer is running out of the robot - Yoshi outtakes */
    OUTTAKE,
    /**
     * Intake is idle - Indexer is running into the robot (shooter wheels) - Yoshi is idle at
     * setpoint
     */
    INDEX,
    /** Intake is idle - Indexer is running out of the robot - Yoshi is idle at setpoint */
    OUTDEX,
    /**
     * Intake is running into the robot - Indexer is running into the robot (shooter wheels) - Yoshi
     * is idle at setpoint
     */
    FIRE,
    /**
     * Intake is idle - Indexer is idle - Yoshi is at ground setpoint spinning flywheels into the
     * amp compartment
     */
    INTAKE_AMP,
    /**
     * Intake is idle - Indexer is idle - Yoshi is at amp setpoint and the amp compartment is
     * spinning outwards into the amp
     */
    SCORE_AMP,
  }
}
