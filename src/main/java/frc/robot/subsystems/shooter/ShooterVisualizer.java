// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

/** Class to visualize the shooter as a mechanism */
public class ShooterVisualizer {
  private Mechanism2d mechanismField = new Mechanism2d(1, 1);
  private MechanismRoot2d armRoot = mechanismField.getRoot("anglerPivot", 0.0, 0.0);
  private final MechanismLigament2d armPivot =
      armRoot.append(
          new MechanismLigament2d(
              "angler", 0.4958, Math.toRadians(0), 10, new Color8Bit(255, 0, 0)));

  public ShooterVisualizer() {}

  public void updateShooterAngle(Rotation2d angle) {
    armPivot.setAngle(angle);
    Logger.recordOutput("Angler/Visualizer", mechanismField);
  }
}
