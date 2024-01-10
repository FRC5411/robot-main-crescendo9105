// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;

/** Swerve drive */
public class Drive extends SubsystemBase {
  // TODO Adjust values as needed
  private final double TRACK_WIDTH_X_M = Units.inchesToMeters(26.0);
  private final double TRACK_WIDTH_Y_M = Units.inchesToMeters(26.0);
  // private final double DRIVEBASE_RADIUS_M =
  //     Math.hypot(TRACK_WIDTH_X_M / 2.0, TRACK_WIDTH_Y_M / 2.0);
  // private final double MAX_SPEED_MPS = Units.feetToMeters(14.0);
  // private final double MAX_ANGULAR_SPEED_MPS = MAX_SPEED_MPS / DRIVEBASE_RADIUS_M;

  private final SwerveDriveKinematics KINEMATICS = getKinematics();

  public static final Lock odometryLock = new ReentrantLock();

  private GyroIO gyroIO;
  private GyroIOInputsAutoLogged gyroIOInputs = new GyroIOInputsAutoLogged();

  private Module[] modules = new Module[4]; // FL FR BL BR

  private Pose2d currentPose = new Pose2d();
  private Rotation2d lastGyroRotation = new Rotation2d();

  // TODO Add poseEstimator

  /** Creates a new swerve Drive. */
  public Drive(
      ModuleIO moduleFL, ModuleIO moduleFR, ModuleIO moduleBL, ModuleIO moduleBR, GyroIO gyro) {
    modules[0] = new Module(moduleFL, 0);
    modules[1] = new Module(moduleFL, 1);
    modules[2] = new Module(moduleFL, 2);
    modules[3] = new Module(moduleFL, 3);
    gyroIO = gyro;

    // TODO Configure PathPlanner here
  }

  @Override
  public void periodic() {
    // Lock odometry while reading data to logger
    odometryLock.lock();
    gyroIO.updateInputs(gyroIOInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    // Now unlock odometry since data has been logged
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroIOInputs);

    for (var module : modules) {
      module.periodic();
    }

    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
        module.setBrake(true); // Apply brakes if disabled (Remember autonomous in the LGI lol)
      }
      // Log empty states
      Logger.recordOutput("Drive/SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Calculate current pose from deltas
    int deltaCount = gyroIOInputs.connected ? gyroIOInputs.odometryYawPositions.length : Integer.MAX_VALUE;
    for (int i = 0; i < 4; i++) {
      deltaCount = Math.min(deltaCount, modules[i].getModuleDeltas().length);
    }
    // Iterate through all of the deltas for this cycle
    for (int deltaIndex = 0; deltaIndex < deltaCount; deltaIndex++) {
      // Get wheel deltas
      SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        wheelDeltas[moduleIndex] = modules[moduleIndex].getModuleDeltas()[moduleIndex]; 
      }

      // Twist is the motion of the robot (x, y, theta) since the last cycle
      var twist = KINEMATICS.toTwist2d(wheelDeltas);
      if (gyroIOInputs.connected) {
        // If gyro is connected, replace the estimated theta with gyro yaw
        Rotation2d gyroRotation = gyroIOInputs.odometryYawPositions[deltaIndex];
        twist = new Twist2d(twist.dx, twist.dy, gyroRotation.minus(lastGyroRotation).getRadians());
        lastGyroRotation = gyroRotation;
      }
      // Apply the change since last sample to current pose
      currentPose = currentPose.exp(twist);
    }
  }

  // TODO Add the rest of the getter and setter methods

  public SwerveDriveKinematics getKinematics() {
    return new SwerveDriveKinematics(
        new Translation2d[] {
          new Translation2d(TRACK_WIDTH_X_M / 2.0, TRACK_WIDTH_Y_M / 2.0),
          new Translation2d(TRACK_WIDTH_X_M / 2.0, -TRACK_WIDTH_Y_M / 2.0),
          new Translation2d(-TRACK_WIDTH_X_M / 2.0, TRACK_WIDTH_Y_M / 2.0),
          new Translation2d(-TRACK_WIDTH_X_M / 2.0, -TRACK_WIDTH_Y_M / 2.0)
        });
  }
}
