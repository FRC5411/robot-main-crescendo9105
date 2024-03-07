// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

/** Class to visualize the shooter as a mechanism */
public class ShooterVisualizer {
  private final String ANGLER_LOG_KEY = "Shooter/Visualizer/Angler";

  private Mechanism2d anglerMechanismVisual = new Mechanism2d(1.0, 1.0);
  private MechanismRoot2d anglerPivotVisual =
      anglerMechanismVisual.getRoot("ANGLER_PIVOT", 0.5, 0.0);
  private MechanismLigament2d shooterVisual =
      anglerPivotVisual.append(new MechanismLigament2d("SHOOTER", 0.4958, Math.toRadians(0.0)));

  /** Create a new visualizer */
  public ShooterVisualizer() {
    shooterVisual.setColor(new Color8Bit(Color.kWhite));

    Logger.recordOutput(ANGLER_LOG_KEY, anglerMechanismVisual);
  }

  /** Update the shooter visualizer */
  public void updateShooterAngle(Rotation2d angle) {
    if (angle == null) {
      shooterVisual.setAngle(Rotation2d.fromDegrees(25.0));
    } else {
      shooterVisual.setAngle(angle);
    }

    Logger.recordOutput(ANGLER_LOG_KEY, anglerMechanismVisual);
  }
}
