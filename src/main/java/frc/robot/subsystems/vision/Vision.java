// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO cameraLeft;
  private final VisionIO cameraRight;

  private final VisionIOInputsAutoLogged inputsRight = new VisionIOInputsAutoLogged();
  private final VisionIOInputsAutoLogged inputsLeft = new VisionIOInputsAutoLogged();

  public Vision() {
    cameraLeft = new VisionIOPhotonVision("LLLeft", new Transform3d(), 0.1);
    cameraRight = new VisionIOPhotonVision("LLRight", new Transform3d(), 0.1);
  }

  @Override
  public void periodic() {
    cameraLeft.updateInputs(inputsLeft);
    cameraRight.updateInputs(inputsRight);

    Logger.processInputs("Vision/Right", inputsRight);
    Logger.processInputs("Vision/Left", inputsLeft);
  }
}
