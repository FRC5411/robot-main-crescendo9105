// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.utils.LoggedTunableNumber;
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
  private SwerveModulePosition[] positionDeltas =
      new SwerveModulePosition[] {}; // Change since last cycle

  private PIDController driveController;
  private PIDController azimuthController;
  private SimpleMotorFeedforward driveFeedforward;

  private LoggedTunableNumber driveControllerP =
      new LoggedTunableNumber("Drive/Module/DriveP", 0.02);
  private LoggedTunableNumber driveControllerI =
      new LoggedTunableNumber("Drive/Module/DriveI", 0.0);
  private LoggedTunableNumber driveControllerD =
      new LoggedTunableNumber("Drive/Module/DriveD", 0.0);

  private LoggedTunableNumber azimuthControllerP =
      new LoggedTunableNumber("Drive/Module/AzimuthP", 2.8);
  private LoggedTunableNumber azimuthControllerI =
      new LoggedTunableNumber("Drive/Module/AzimuthI", 0.0);
  private LoggedTunableNumber azimuthControllerD =
      new LoggedTunableNumber("Drive/Module/AzimuthD", 0.00001);

  /** Creates a new swerve module */
  public Module(ModuleIO io, int id) {
    moduleIO = io;
    MODULE_ID = id;

    switch (Constants.currentMode) {
      case REAL:
        driveController = new PIDController(0.02, 0.0, 0.0);
        azimuthController = new PIDController(2.8, 0.0, 0.00001);
        driveFeedforward = new SimpleMotorFeedforward(0.0, 0.21);
        break;
      case REPLAY:
        driveController = new PIDController(0.05, 0.0, 0.0);
        azimuthController = new PIDController(7.0, 0.0, 0.0);
        driveFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
        break;
      case SIM:
        driveController = new PIDController(0.1, 0.0, 0.0);
        azimuthController = new PIDController(10.0, 0.0, 0.0);
        driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
        break;
      default:
        driveController = new PIDController(0.0, 0.0, 0.0);
        azimuthController = new PIDController(0.0, 0.0, 0.0);
        driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

    azimuthController.enableContinuousInput(-Math.PI, Math.PI);
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

    // Run setpoints for modules
    if (angleSetpoint != null) {
      moduleIO.setAzimuthVolts(
          azimuthController.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));

      if (velocitySetpoint != null) {
        double adjustedVelocitySetpoint =
            velocitySetpoint * Math.cos(azimuthController.getPositionError());

        double velocityMPS = adjustedVelocitySetpoint / WHEEL_RADIUS_M;
        moduleIO.setDriveVolts(
            driveFeedforward.calculate(velocityMPS)
                + driveController.calculate(moduleIOInputs.driveVelocityMPS, velocityMPS));
      }
    }

    // Calculate deltas (change) from odometry
    int deltaCount = // deltaCount based on how many frames were captured
        Math.min(
            moduleIOInputs.odometryDrivePositionM.length,
            moduleIOInputs.odometryAzimuthPositions.length);
    positionDeltas =
        new SwerveModulePosition
            [deltaCount]; // Array resets every loop to account for varying frames
    for (int i = 0; i < deltaCount; i++) {
      double positionM = moduleIOInputs.odometryDrivePositionM[i] * WHEEL_RADIUS_M;
      Rotation2d angle =
          moduleIOInputs.odometryAzimuthPositions[i].plus(
              azimuthRelativeOffset != null ? azimuthRelativeOffset : new Rotation2d());

      positionDeltas[i] =
          new SwerveModulePosition(
              positionM - lastPositionM, angle); // Update position from odometry
      lastPositionM = positionM;
    }

    if (driveControllerP.hasChanged(hashCode())
        || driveControllerI.hasChanged(hashCode())
        || driveControllerD.hasChanged(hashCode())) {
      driveController.setP(driveControllerP.get());
      driveController.setI(driveControllerI.get());
      driveController.setD(driveControllerD.get());
    }
    if (azimuthControllerP.hasChanged(hashCode())
        || azimuthControllerI.hasChanged(hashCode())
        || azimuthControllerD.hasChanged(hashCode())) {
      azimuthController.setP(azimuthControllerP.get());
      azimuthController.setI(azimuthControllerI.get());
      azimuthController.setD(azimuthControllerD.get());
    }
  }

  // TODO Remove after testing
  public void setVolts(double volts) {
    moduleIO.setDriveVolts(volts);
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

  /** Get the module position deltas from this cycle */
  public SwerveModulePosition[] getModuleDeltas() {
    return positionDeltas;
  }
}
