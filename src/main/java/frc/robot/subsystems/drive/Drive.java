// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Swerve drive */
public class Drive extends SubsystemBase {
  // TODO Adjust values as needed
  private final double TRACK_WIDTH_X_M = Units.inchesToMeters(26.0);
  private final double TRACK_WIDTH_Y_M = Units.inchesToMeters(26.0);
  private final double DRIVEBASE_RADIUS_M =
      Math.hypot(TRACK_WIDTH_X_M / 2.0, TRACK_WIDTH_Y_M / 2.0);
  private final double MAX_LINEAR_SPEED_MPS = Units.feetToMeters(14.0);
  private final double MAX_ANGULAR_SPEED_MPS = MAX_LINEAR_SPEED_MPS / DRIVEBASE_RADIUS_M;

  private final SwerveDriveKinematics KINEMATICS = getKinematics();

  // Odometry threads whine when I remove this so I keep for now
  public static final Lock odometryLock = new ReentrantLock();

  private GyroIO gyroIO;
  private GyroIOInputsAutoLogged gyroIOInputs = new GyroIOInputsAutoLogged();

  private Module[] modules = new Module[4]; // FL FR BL BR

  private Pose2d currentPose = new Pose2d();

  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(KINEMATICS, getRotation(), getModulePositions(), currentPose);

  /** Creates a new swerve Drive. */
  public Drive(
      ModuleIO moduleFL, ModuleIO moduleFR, ModuleIO moduleBL, ModuleIO moduleBR, GyroIO gyro) {
    modules[0] = new Module(moduleFL, 0);
    modules[1] = new Module(moduleFR, 1);
    modules[2] = new Module(moduleBL, 2);
    modules[3] = new Module(moduleBR, 3);
    gyroIO = gyro;

    // Configure PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPosition,
        this::setPose,
        () -> KINEMATICS.toChassisSpeeds(getModuleStates()),
        this::runSwerve,
        new HolonomicPathFollowerConfig(
            MAX_LINEAR_SPEED_MPS, DRIVEBASE_RADIUS_M, new ReplanningConfig(true, true)),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Drive/Odometry/Trajectory",
              activePath.toArray(new Pose2d[activePath.size()])); // Autolog the trajectory
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput(
              "Drive/Odometry/TrajectorySetpoint", targetPose); // Auto log the target setpoint
        });
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
      }
      // Log empty states
      Logger.recordOutput("Drive/SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }
    if (DriverStation.isEStopped()) {
      for (var module : modules) {
        module.stop();
        module.setBrake(true); // Apply brakes if E-Stopped
      }
    }

    poseEstimator.update(getRotation(), getModulePositions());

    currentPose = poseEstimator.getEstimatedPosition();
  }

  /** Runs the swerve drive based on speeds */
  public void runSwerve(ChassisSpeeds speeds) {
    // Calculates setpoint states from inputs
    ChassisSpeeds discreteSpeeds =
        ChassisSpeeds.discretize(speeds, 0.02); // Compensate for translational skew when rotating
    SwerveModuleState[] setpointStates = KINEMATICS.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, MAX_LINEAR_SPEED_MPS); // Normalize speeds

    // Set and log the optimized states
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      optimizedSetpointStates[i] =
          modules[i].setDesiredState(
              setpointStates[i]); // setDesiredState returns the optimized state
    }

    Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive */
  public void stop() {
    runSwerve(new ChassisSpeeds());
  }

  public void resetPose() {
    poseEstimator.resetPosition(gyroIOInputs.yawPosition, getModulePositions(), new Pose2d());
  }

  /** Set the pose of the robot */
  public void setPose(Pose2d pose) {
    currentPose = pose;
  }

  /** Add a vision measurement for the poseEstimator */
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampS) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampS);
  }

  /** Get PathFinder constraints */
  public PathConstraints getPathConstraints() {
    return new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
  }

  /** Gets the PathFinder setpoint, you can set the setpoint in SmartDashboard */
  @AutoLogOutput(key = "Drive/PathFinder/Setpoint")
  public Pose2d getPathFinderSetpoint() {
    // System.out.println("getPathFinderSetpoint");

    return new Pose2d(
        SmartDashboard.getNumber("PathfindX", 0.0),
        SmartDashboard.getNumber("PathfindY", 0.0),
        new Rotation2d());
  }

  /** Gets the drive's measured state (module azimuth angles and drive velocities) */
  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getModuleState();
    }

    return states;
  }

  /** Gets the swerve module's positions */
  @AutoLogOutput(key = "Drive/ModulePositions")
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] =
          modules[i] != null ? modules[i].getModulePosition() : new SwerveModulePosition();
    }

    return positions;
  }

  // TODO Fuse 250hz odometry with vision
  /** Gets the pose of the robot */
  @AutoLogOutput(key = "Drive/Odometry/Pose")
  public Pose2d getPosition() {
    return currentPose;
  }

  /** Gets the rotation of the robot */
  @AutoLogOutput(key = "Drive/Odometry/Rotation")
  public Rotation2d getRotation() {
    return gyroIOInputs.yawPosition;
  }

  public double getMaxLinearSpeedMPS() {
    return MAX_LINEAR_SPEED_MPS;
  }

  public double getMaxAngularSpeedMPS() {
    return MAX_ANGULAR_SPEED_MPS;
  }

  /** Gets the kinematics of the drivetrain */
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
