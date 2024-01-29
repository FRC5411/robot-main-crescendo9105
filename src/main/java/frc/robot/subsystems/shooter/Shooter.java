// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.Flywheel.ShooterWheelIO;
import frc.robot.subsystems.shooter.Flywheel.ShooterWheelSimIO;
import frc.robot.subsystems.shooter.Indexer.IndexerIO;
import frc.robot.subsystems.shooter.Indexer.IndexerSim;
import frc.robot.subsystems.shooter.LeadScrewArm.ScrewArmIO;
import frc.robot.subsystems.shooter.LeadScrewArm.ScrewArmSim;

public class Shooter extends SubsystemBase {

  public Shooter() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
