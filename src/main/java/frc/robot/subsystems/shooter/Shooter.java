// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.Flywheel.ShooterWheelConstants;
import frc.robot.subsystems.shooter.Flywheel.ShooterWheelIO;
import frc.robot.subsystems.shooter.Flywheel.ShooterWheelSimIO;
import frc.robot.subsystems.shooter.Flywheel.ShooterWheelTalonFX;
import frc.robot.subsystems.shooter.Indexer.Indexer550;
import frc.robot.subsystems.shooter.Indexer.IndexerIO;
import frc.robot.subsystems.shooter.Indexer.IndexerSim;
import frc.robot.subsystems.shooter.LeadScrewArm.ScrewArmConstants;
import frc.robot.subsystems.shooter.LeadScrewArm.ScrewArmIO;
import frc.robot.subsystems.shooter.LeadScrewArm.ScrewArmNEO;
import frc.robot.subsystems.shooter.LeadScrewArm.ScrewArmSim;

public class Shooter extends SubsystemBase {
  private ShooterWheelIO shooterWheelTop;
  private ShooterWheelIO shooterWheelBottom;
  private IndexerIO indexerIO;
  private ScrewArmIO screwArmIO;

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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
