// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

/** Class to visualize the climbs as a mechanism */
public class ClimbVisualizer {
  private final climbSide NAME;

  private Mechanism2d climbMechanism = new Mechanism2d(3, 2);

  private MechanismRoot2d climbPivot =
      climbMechanism.getRoot("climbPivot", 1.5, Units.inchesToMeters(21.0));

  private MechanismLigament2d climbTower =
      climbPivot.append(new MechanismLigament2d("climbTower", 21.0, -90.0));
  private MechanismLigament2d climbArm =
      climbPivot.append(
          new MechanismLigament2d(
              "climbArm", Units.inchesToMeters(12.5), 0, 6.0, new Color8Bit(Color.kWhite)));

  /** Creates a new visual for the climb */
  public ClimbVisualizer(climbSide arm) {
    NAME = arm;

    climbTower.setColor(new Color8Bit(Color.kAqua));
    climbTower.setLineWeight(5.0);

    Logger.recordOutput("Climb/" + NAME, climbMechanism);
  }

  /** Update the angle of the climb arm */
  public void updateClimbAngle(Rotation2d angleRadians) {
    climbArm.setAngle(angleRadians);

    Logger.recordOutput("Climb/" + NAME, climbMechanism);
  }

  /** Which side this climb is on */
  public enum climbSide {
    LEFT,
    RIGHT
  }
}
