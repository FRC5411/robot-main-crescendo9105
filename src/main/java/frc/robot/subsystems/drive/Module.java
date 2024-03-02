// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

/** Swerve module wrapper */
public class Module {
  public static final double ODOMETRY_FREQUENCY = 250.0;
  private final int MODULE_ID;

  private ModuleIO moduleIO;
  private ModuleIOInputsAutoLogged moduleIOInputs = new ModuleIOInputsAutoLogged();

  Double velocitySetpoint = null;
  Rotation2d angleSetpoint = null;

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

    // Run PID (in IO layer)
    if (angleSetpoint != null) {
      moduleIO.setAzimuthPosition(angleSetpoint);
    }
    if (velocitySetpoint != null) {
      moduleIO.setDriveVelocity(velocitySetpoint);
    }
  }

  /** Sets the module's state */
  public SwerveModuleState setDesiredState(SwerveModuleState desiredState) {
    var optimizedState = desiredState; // SwerveModuleState.optimize(desiredState, getAngle());

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
    return moduleIOInputs.azimuthPosition;
  }

  /** Get the distance travelled by the drive motor */
  public double getPositionM() {
    return moduleIOInputs.drivePositionM;
  }

  /** Get the velocity of the drive motor */
  public double getVelocityMPS() {
    return moduleIOInputs.driveVelocityMPS;
  }

  /** Get the module's distance and angle */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(getPositionM(), getAngle());
  }

  /** Get the module's state */
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(getVelocityMPS(), getAngle());
  }
}
