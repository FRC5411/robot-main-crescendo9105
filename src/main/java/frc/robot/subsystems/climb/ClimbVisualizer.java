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

public class ClimbVisualizer {
  private final ClimbSide NAME;

  private Mechanism2d climbMechanism = new Mechanism2d(3, 2);

  private MechanismRoot2d climbPivot =
      climbMechanism.getRoot("ClimbPivot", 1.5, Units.inchesToMeters(21.0));

  private MechanismLigament2d climbTower =
      climbPivot.append(new MechanismLigament2d("ClimbTower", 21.0, -90.0));
  private MechanismLigament2d climbArm =
      climbPivot.append(new MechanismLigament2d("ClimbArm", Units.inchesToMeters(12.5), 0.0));

  public ClimbVisualizer(ClimbSide arm) {
    NAME = arm;

    climbTower.setColor(new Color8Bit(Color.kAqua));
    climbTower.setLineWeight(5.0);

    if (NAME == ClimbSide.LEFT)       climbArm.setColor(new Color8Bit(Color.kWhite));
    else if (NAME == ClimbSide.RIGHT) climbArm.setColor(new Color8Bit(Color.kBlue));
    
    climbArm.setLineWeight(5.0);

    Logger.recordOutput("Climb/Visualizer/" + NAME, climbMechanism);
  }

  public void updateClimbAngle(Rotation2d armPosition) {
    if (armPosition == null) climbArm.setAngle(new Rotation2d());
    else climbArm.setAngle(armPosition);

    Logger.recordOutput("Climb/Visualizer/" + NAME, climbMechanism);
  }

  public enum ClimbSide {
    LEFT,
    RIGHT
  }
}
