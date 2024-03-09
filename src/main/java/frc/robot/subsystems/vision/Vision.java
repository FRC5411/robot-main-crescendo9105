// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO cameraLeft;
  private final VisionIO cameraRight;

  private final VisionIOInputsAutoLogged inputsRight = new VisionIOInputsAutoLogged();
  private final VisionIOInputsAutoLogged inputsLeft = new VisionIOInputsAutoLogged();

  public Vision(VisionIO left, VisionIO right) {
    cameraLeft = left;
    cameraRight = right;
  }

  @Override
  public void periodic() {
    cameraLeft.updateInputs(inputsLeft);
    cameraRight.updateInputs(inputsRight);

    Logger.processInputs("Vision/Right", inputsRight);
    Logger.processInputs("Vision/Left", inputsLeft);
  }

  public void setSingleStdDevs(double x, double y, double theta) {
    cameraLeft.setSingleStdDevs(x, y, theta);
    cameraRight.setSingleStdDevs(x, y, theta);
  }

  public void setMultiStdDevs(double x, double y, double theta) {
    cameraLeft.setMultiStdDevs(x, y, theta);
    cameraRight.setMultiStdDevs(x, y, theta);
  }

  public VisionIOInputsAutoLogged getInputsRight() {
    return inputsRight;
  }

  public VisionIOInputsAutoLogged getInputsLeft() {
    return inputsLeft;
  }

  public Matrix<N3, N1> getSingleStdDevsCoeff() {
    return cameraLeft.getSingleStdDevsCoeff();
  }

  public Matrix<N3, N1> getMultiStdDevsCoeff() {
    return cameraLeft.getMultiStdDevsCoeff();
  }
}
