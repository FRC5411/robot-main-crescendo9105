// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

/** Swerve module wrapper */
public class Module {
  // TODO Double check wheel radius
  private final double WHEEL_RADIUS_M = Units.inchesToMeters(3.0);
  public static final double ODOMETRY_FREQUENCY = 250.0;
  private final int MODULE_ID;

  private ModuleIO moduleIO;
  private ModuleIOInputsAutoLogged moduleIOInputs = new ModuleIOInputsAutoLogged();

  private Double velocitySetpoint = null;
  private Rotation2d angleSetpoint = null;

  private Rotation2d azimuthRelativeOffset = null;
  private double lastPositionM = 0.0;
  private SwerveModulePosition[] positionDeltas = new SwerveModulePosition[] {}; // Change since last cycle

  /** Creates a new swerve module */
  public Module(ModuleIO io, int id) {
    moduleIO = io;
    MODULE_ID = id;
  }

  /**
   * Update inputs without running periodic logic; Odometry updates need to be properly thread
   * locked
   */
  public void updateInputs() {
    moduleIO.updateInputs(moduleIOInputs);
  }

  /** Called in subsystem periodic */
  public void periodic() {
    Logger.processInputs("Drive/Module" + Integer.toString(MODULE_ID), moduleIOInputs);

    // Reset relative encoder on first cycle
    if (azimuthRelativeOffset == null
        && moduleIOInputs.azimuthAbsolutePosition.getRadians() != 0.0) {
      azimuthRelativeOffset =
          moduleIOInputs.azimuthAbsolutePosition.minus(moduleIOInputs.azimuthPosition);
    }

    // Run closed loop control for azimuth
    if (angleSetpoint != null) {
      moduleIO.setAngle(angleSetpoint.getRadians());

      // TODO See if we can optimize the velocity setpoint to increase as the wheel moves
      //      closer to its goal
      moduleIO.setVelocity(velocitySetpoint);
    }

    // Calculate deltas (change) from odometry
    int deltaCount = // deltaCount based on how many frames were captured
        Math.min(
            moduleIOInputs.odometryDrivePositionR.length,
            moduleIOInputs.odometryAzimuthPositions.length);
    positionDeltas = new SwerveModulePosition[deltaCount]; // Array resets every loop to account for varying frames
    for (int i = 0; i < deltaCount; i++) {
      double positionM = moduleIOInputs.odometryDrivePositionR[i] * WHEEL_RADIUS_M;
      Rotation2d angle =
          moduleIOInputs.odometryAzimuthPositions[i].plus(
              azimuthRelativeOffset != null ? azimuthRelativeOffset : new Rotation2d());
              
      positionDeltas[i] = new SwerveModulePosition(positionM - lastPositionM, angle); // Update position from odometry
      lastPositionM = positionM;
    }
  }

  /** Sets the module's state */
  public SwerveModuleState setDesiredState(SwerveModuleState desiredState) {
    // TODO Update for get angle
    var optimizedState = SwerveModuleState.optimize(desiredState, getAngle());

    // Controllers run in IO, which is called in periodic
    velocitySetpoint = optimizedState.speedMetersPerSecond;
    angleSetpoint = optimizedState.angle;

    return optimizedState;
  }

  // TODO Add characterization methods

  /** Disables all motor outputs */
  public void stop() {
    moduleIO.setDriveVolts(0.0);
    moduleIO.setAzimuthVolts(0.0);

    // null prevents controllers from running
    velocitySetpoint = null;
    angleSetpoint = null;
  }

  /** Sets the module's IdleMode */
  public void setBrake(boolean shouldBrake) {
    moduleIO.setDriveBrake(shouldBrake);
    moduleIO.setAzimuthBrake(shouldBrake);
  }

  /** Get the current angle of the azimuth */
  public Rotation2d getAngle() {
    if (azimuthRelativeOffset == null) {
      return new Rotation2d();
    } else {
      // Add the calculated offset to the relative encoder's reading
      return moduleIOInputs.azimuthPosition.plus(azimuthRelativeOffset);
    }
  }

  /** Get the distance travelled by the drive motor */
  public double getPositionM() {
    return moduleIOInputs.drivePositionR * WHEEL_RADIUS_M;
  }

  /** Get the velocity of the drive motor */
  public double getVelocityMPS() {
    return moduleIOInputs.driveVelocityRPS * WHEEL_RADIUS_M;
  }

  /** Get the module's distance and angle */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(getPositionM(), getAngle());
  }

  /** Get the module's state */
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(getVelocityMPS(), getAngle());
  }

  /** Get the module position deltas from this cycle */
  public SwerveModulePosition[] getModuleDeltas() {
    return positionDeltas;
  }
}
