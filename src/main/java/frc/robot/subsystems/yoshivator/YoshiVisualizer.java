// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.yoshivator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

// /** Class to visualize the shooter as a mechanism */
// public class ShooterVisualizer {
//   private final String ANGLER_LOG_KEY = "Shooter/Visualizer/Angler";

//   private Mechanism2d anglerMechanismVisual = new Mechanism2d(1.0, 1.0);
//   private MechanismRoot2d anglerPivotVisual =
//       anglerMechanismVisual.getRoot("ANGLER_PIVOT", 0.5, 0.0);
//   private MechanismLigament2d shooterVisual;

//   /** Create a new visualizer */
//   public ShooterVisualizer(Rotation2d initialAngle) {
//     shooterVisual =
//         anglerPivotVisual.append(
//             new MechanismLigament2d("SHOOTER", 0.4958, initialAngle.getRadians()));
//     shooterVisual.setColor(new Color8Bit(Color.kWhite));

//     Logger.recordOutput(ANGLER_LOG_KEY, anglerMechanismVisual);
//   }

//   /** Update the shooter visualizer */
//   public void updateShooterAngle(Rotation2d angle) {
//     if (angle == null) {
//       shooterVisual.setAngle(Rotation2d.fromDegrees(25.0));
//     } else {
//       shooterVisual.setAngle(angle.minus(Rotation2d.fromDegrees(180.0)).times(-1.0));
//     }

//     Logger.recordOutput(ANGLER_LOG_KEY, anglerMechanismVisual);
//   }
// }

/** Class to visualize the yoshi as a mechanism */
public class YoshiVisualizer {
  private final String PIVOT_LOG_KEY = "Yoshivator/Visualizer/Pivot";

  private Mechanism2d pivotMechanismVisual = new Mechanism2d(1.0, 1.0);
  private MechanismRoot2d pivotPointVisual = pivotMechanismVisual.getRoot("YOSHI_PIVOT", 0.5, 0.0);
  private MechanismLigament2d pivotVisual;

  /** Create a new visualizer */
  public YoshiVisualizer(Rotation2d initialAngle) {
    pivotVisual =
        pivotPointVisual.append(new MechanismLigament2d("YOSHI", 0.5, initialAngle.getRadians()));
    pivotVisual.setColor(new Color8Bit(Color.kWhite));

    Logger.recordOutput(PIVOT_LOG_KEY, pivotMechanismVisual);
  }

  /** Update the yoshi visualizer */
  public void updateYoshiAngle(Rotation2d angle) {
    if (angle == null) {
      pivotVisual.setAngle(new Rotation2d());
    } else {
      pivotVisual.setAngle(angle);
    }

    Logger.recordOutput(PIVOT_LOG_KEY, pivotMechanismVisual);
  }
}
