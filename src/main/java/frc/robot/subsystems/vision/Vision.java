// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2024-crescendo.json
// Apriltag map being used currently, can be cropped to only include the tags being used if
// necessary

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;

public class Vision extends SubsystemBase {
  private final VisionIO cameraLeft;
  private final VisionIO cameraRight;

  private final VisionIOInputsAutoLogged inputsRight = new VisionIOInputsAutoLogged();
  private final VisionIOInputsAutoLogged inputsLeft = new VisionIOInputsAutoLogged();

  private Matrix<N3, N1> singleTagStdDevs;
  private Matrix<N3, N1> multiTagStdDevs;

  public Vision(VisionIO left, VisionIO right) {
    cameraLeft = left;
    cameraRight = right;

    singleTagStdDevs = VecBuilder.fill(0.07, 0.07, Double.MAX_VALUE);
    multiTagStdDevs = VecBuilder.fill(0.04, 0.04, Double.MAX_VALUE);
  }

  @Override
  public void periodic() {
    cameraLeft.updateInputs(inputsLeft);
    cameraRight.updateInputs(inputsRight);

    Logger.processInputs("Vision/Right", inputsRight);
    Logger.processInputs("Vision/Left", inputsLeft);
  }

  public void setSingleStdDevsCoeff(double x, double y, double theta) {
    singleTagStdDevs = VecBuilder.fill(x, y, theta);
  }

  public void setMultiStdDevsCoeff(double x, double y, double theta) {
    multiTagStdDevs = VecBuilder.fill(x, y, theta);
  }

  public Matrix<N3, N1> getSingleStdDevsCoeff() {
    return singleTagStdDevs;
  }

  public Matrix<N3, N1> getMultiStdDevsCoeff() {
    return multiTagStdDevs;
  }

  public Matrix<N3, N1> getLeftStdDevs() {
    return getEstimationStdDevs(inputsLeft.numberOfTargets, inputsLeft.averageDistanceFromTagMeters);
  }

  public Matrix<N3, N1> getRightStdDevs() {
    return getEstimationStdDevs(inputsRight.numberOfTargets, inputsRight.averageDistanceFromTagMeters);
  }

  private Matrix<N3, N1> getEstimationStdDevs(int numTags, double avgDist) {
    Matrix<N3, N1> estStdDevs = singleTagStdDevs;

    // No tags visible
    if (numTags == 0) return estStdDevs;

    // Calculate average distance
    avgDist /= numTags;

    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = multiTagStdDevs;

    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 3) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }
    else if (avgDist > 5.5) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }
    else estStdDevs = estStdDevs.times((1 + (avgDist * avgDist / 30)) / (double) numTags);

    return estStdDevs;
  }

  public VisionIOInputsAutoLogged getInputsRight() {
    return inputsRight;
  }

  public VisionIOInputsAutoLogged getInputsLeft() {
    return inputsLeft;
  }
}
