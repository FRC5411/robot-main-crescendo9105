// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.Flywheel.ShooterWheelConstants;
import frc.robot.subsystems.shooter.Flywheel.ShooterWheelIO;
import frc.robot.subsystems.shooter.Flywheel.ShooterWheelIOInputsAutoLogged;
import frc.robot.subsystems.shooter.Flywheel.ShooterWheelSimIO;
import frc.robot.subsystems.shooter.Flywheel.ShooterWheelTalonFX;
import frc.robot.subsystems.shooter.Indexer.Indexer550;
import frc.robot.subsystems.shooter.Indexer.IndexerIO;
import frc.robot.subsystems.shooter.Indexer.IndexerIOInputsAutoLogged;
import frc.robot.subsystems.shooter.Indexer.IndexerSim;
import frc.robot.subsystems.shooter.LeadScrewArm.ScrewArmConstants;
import frc.robot.subsystems.shooter.LeadScrewArm.ScrewArmIO;
import frc.robot.subsystems.shooter.LeadScrewArm.ScrewArmInputsAutoLogged;
import frc.robot.subsystems.shooter.LeadScrewArm.ScrewArmNEO;
import frc.robot.subsystems.shooter.LeadScrewArm.ScrewArmSim;

public class Shooter extends SubsystemBase {
  private ShooterWheelIO shooterWheelTop;
  private ShooterWheelIO shooterWheelBottom;
  private IndexerIO indexerIO;
  private ScrewArmIO screwArmIO;

  private Subsystem shooterWheelSub = new Subsystem() {};
  private Subsystem indexerSub = new Subsystem() {};
  private Subsystem screwArmSub = new Subsystem() {};

  private double topVelocityMPS = 0;
  private double bottomVelocityMPS = 0;

  private double indexerVoltage = 0;

  private ShooterWheelIOInputsAutoLogged shooterWheelIOInputsAutoLogged = new ShooterWheelIOInputsAutoLogged();
  private ScrewArmInputsAutoLogged screwArmInputsAutoLogged = new ScrewArmInputsAutoLogged();
  private IndexerIOInputsAutoLogged indexerIOInputsAutoLogged = new IndexerIOInputsAutoLogged();

  public Shooter() {
    if(RobotBase.isReal()) {
      shooterWheelTop = new ShooterWheelTalonFX(
        ShooterWheelConstants.kTopMotorID, false,
        new PIDController(
          ShooterWheelConstants.kP,
          ShooterWheelConstants.kI,
          ShooterWheelConstants.kD),
        new SimpleMotorFeedforward(
          ShooterWheelConstants.kS,
          ShooterWheelConstants.kV,
          ShooterWheelConstants.kA),
        ShooterWheelConstants.kFlywheelRateLimit);
      shooterWheelBottom = new ShooterWheelTalonFX(
        ShooterWheelConstants.kBottomMotorID, false,
        new PIDController(
          ShooterWheelConstants.kP,
          ShooterWheelConstants.kI,
          ShooterWheelConstants.kD),
        new SimpleMotorFeedforward(
          ShooterWheelConstants.kS,
          ShooterWheelConstants.kV,
          ShooterWheelConstants.kA),
        ShooterWheelConstants.kFlywheelRateLimit);

      indexerIO = new Indexer550();
      screwArmIO = new ScrewArmNEO(ScrewArmConstants.kMotorID);
    } else if(RobotBase.isSimulation()) {
      shooterWheelTop = new ShooterWheelSimIO(
        new PIDController(
          ShooterWheelConstants.kSimP,
          ShooterWheelConstants.kSimI,
          ShooterWheelConstants.kSimD),
        new SimpleMotorFeedforward(
          ShooterWheelConstants.kS,
          ShooterWheelConstants.kV,
          ShooterWheelConstants.kA),
        ShooterWheelConstants.kFlywheelRateLimit);
      shooterWheelBottom = new ShooterWheelSimIO(
        new PIDController(
          ShooterWheelConstants.kSimP,
          ShooterWheelConstants.kSimI,
          ShooterWheelConstants.kSimD),
        new SimpleMotorFeedforward(
          ShooterWheelConstants.kS,
          ShooterWheelConstants.kV,
          ShooterWheelConstants.kA),
        ShooterWheelConstants.kFlywheelRateLimit);

      indexerIO = new IndexerSim();
      screwArmIO = new ScrewArmSim();
    } else {
      shooterWheelTop = new ShooterWheelIO() {};
      shooterWheelBottom = new ShooterWheelIO() {};
      indexerIO = new IndexerIO() {};
      screwArmIO = new ScrewArmIO() {};
    }

    shooterWheelSub.setDefaultCommand( shooterVelocityCommand( 
      () -> topVelocityMPS, 
      () -> bottomVelocityMPS ) );


    screwArmSub.setDefaultCommand( anglerPositionCommand() );

    indexerSub.setDefaultCommand( indexerVoltageCommand( () -> indexerVoltage ) );
  }

  public Command setShooterSetpoints(
      double topVelocityMPSSetpoint, double bottomVelocityMPSSetpoint, 
      Rotation2d screwAngleSetpoint, double indexerVoltageSetpoint) {
    return new InstantCommand(
      () -> {
        topVelocityMPS = topVelocityMPSSetpoint;
        bottomVelocityMPS = bottomVelocityMPSSetpoint;

        screwArmIO.setGoal(screwAngleSetpoint);
        screwArmIO.initPID();

        indexerVoltage = indexerVoltageSetpoint;
      }, 
      this);
  }

  public Command setIndexerVoltage(double volts) {
    return new InstantCommand( () -> indexerVoltage = 12, this);
  }

  public Command shooterVelocityCommand(DoubleSupplier topVelocityMPS, DoubleSupplier bottomVelocityMPS) {
    return new FunctionalCommand(
      () -> {}, 
      () -> {
        shooterWheelTop.setFlywheelsVelocity(topVelocityMPS.getAsDouble());
        shooterWheelBottom.setFlywheelsVelocity(bottomVelocityMPS.getAsDouble());
      },
      (interrupted) -> {
        shooterWheelTop.setFlywheelsVolts(0);
        shooterWheelBottom.setFlywheelsVolts(0);
      },
      () -> false, 
      shooterWheelSub);
  }

  public Command anglerPositionCommand() {
    return new FunctionalCommand(
      () -> screwArmIO.initPID(), 
      () -> screwArmIO.executePID(),
      (interrupted) -> screwArmIO.setScrewArmVolts(0),
      () -> false, 
      screwArmSub);
  }

  public Command indexerVoltageCommand(DoubleSupplier voltage) {
    return new FunctionalCommand(
      () -> {}, 
      () -> indexerIO.setIndexerVolts(voltage.getAsDouble()),
      (interrupted) -> indexerIO.setIndexerVolts(0),
      () -> false,
      indexerSub);
  }

  @Override
  public void periodic() {
    shooterWheelTop.updateInputs(shooterWheelIOInputsAutoLogged);
    shooterWheelBottom.updateInputs(shooterWheelIOInputsAutoLogged);
    screwArmIO.updateInputs(screwArmInputsAutoLogged);
    indexerIO.updateInputs(indexerIOInputsAutoLogged);

    Logger.processInputs("Shooter/TopWheel", shooterWheelIOInputsAutoLogged);
    Logger.processInputs("Shooter/BottomWheel", shooterWheelIOInputsAutoLogged);
    Logger.processInputs("Shooter/ScrewArm", screwArmInputsAutoLogged);
    Logger.processInputs("Shooter/Indexer", indexerIOInputsAutoLogged);
  }
}
