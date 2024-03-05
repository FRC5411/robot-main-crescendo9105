// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;

/** Feedforward for a lead-screw arm supported by gas-shocks */
public class ScrewArmFeedforward {

  private double U;
  private double L;

  /**
   * Createa a new Screw Arm feedforward
   *
   * @param kU Upward motion gain
   * @param kL Downward motion gain
   */
  public ScrewArmFeedforward(double kU, double kL) {
    U = kU;
    L = kL;
  }

  /**
   * Calculate the amount of push based off of the arm's angular direction - converts to degrees
   *
   * @param currentPosition The arm's current position
   * @param positionGoal The arm's setpoint position
   * @return The calculated output based off of the two parameters
   */
  public double calculate(Rotation2d currentPosition, Rotation2d positionGoal) {
    double output = 0.0;

    double angleSignum = Math.signum(currentPosition.minus(positionGoal).getDegrees());
    if (angleSignum < 0.0) {
      output = L * angleSignum;
    } else if (angleSignum > 0.0) {
      output = U * angleSignum;
    }

    return output;
  }

  /**
   * Update the U gain
   *
   * @param kU The upward gain to update
   */
  public void updateU(double kU) {
    this.U = kU;
  }

  /**
   * Update the L gain
   *
   * @param kL The downward gain to update
   */
  public void updateL(double kL) {
    this.L = kL;
  }

  /**
   * Get the current upward gain
   *
   * @return The current upward gain
   */
  public double getU() {
    return U;
  }

  /**
   * Get the current downward gain
   *
   * @return The current downward gain
   */
  public double getL() {
    return L;
  }
}
