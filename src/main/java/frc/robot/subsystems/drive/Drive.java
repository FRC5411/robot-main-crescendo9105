// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
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
import frc.robot.utils.Swerve.ModuleLimits;
import frc.robot.utils.Swerve.SwerveSetpoint;
import frc.robot.utils.Swerve.SwerveSetpointGenerator;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Swerve drive */
public class Drive extends SubsystemBase {
  // TODO Adjust values as needed
  private final double TRACK_WIDTH_X_M = Units.inchesToMeters(29.5);
  private final double TRACK_WIDTH_Y_M = Units.inchesToMeters(29.5);
  private final double DRIVEBASE_RADIUS_M =
      Math.hypot(TRACK_WIDTH_X_M / 2.0, TRACK_WIDTH_Y_M / 2.0);
  private final double MAX_LINEAR_SPEED_MPS = Units.feetToMeters(14.0);
  private final double MAX_ANGULAR_SPEED_MPS = MAX_LINEAR_SPEED_MPS / DRIVEBASE_RADIUS_M;
  private ModuleLimits MODULE_LIMITS =
      new ModuleLimits(MAX_LINEAR_SPEED_MPS, MAX_LINEAR_SPEED_MPS * 5, MAX_ANGULAR_SPEED_MPS);
  private SwerveSetpoint currentSetpoint =
      new SwerveSetpoint(
          new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
          });
  private SwerveSetpointGenerator setpointGenerator;

  private final Translation2d[] MODULE_TRANSLATIONS = getModuleTranslations();
  private final SwerveDriveKinematics KINEMATICS = getKinematics();
  private final boolean modulesOrienting = false;

  private GyroIO gyroIO;
  private GyroIOInputsAutoLogged gyroIOInputs = new GyroIOInputsAutoLogged();

  private Module[] modules = new Module[4]; // FL FR BL BR

  private Pose2d currentPose = new Pose2d();

  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(KINEMATICS, getRotation(), getModulePositions(), currentPose);

  private PIDConstants translationPathplannerConstants = new PIDConstants(2.02, 0.0, 0.0);
  private PIDConstants rotationPathplannerConstants = new PIDConstants(0.66, 0.0, 0.0);

  /** Creates a new swerve Drive. */
  public Drive(
      ModuleIO moduleFL, ModuleIO moduleFR, ModuleIO moduleBL, ModuleIO moduleBR, GyroIO gyro) {
    modules[0] = new Module(moduleFL, 0);
    modules[1] = new Module(moduleFR, 1);
    modules[2] = new Module(moduleBL, 2);
    modules[3] = new Module(moduleBR, 3);
    gyroIO = gyro;

    setpointGenerator =
        SwerveSetpointGenerator.builder()
            .kinematics(KINEMATICS)
            .moduleLocations(MODULE_TRANSLATIONS)
            .build();

    Logger.recordOutput("Drive/MaxAngularSpeed", MAX_ANGULAR_SPEED_MPS);
    configDriveAutonomous();
  }

  public void configDriveAutonomous() {
    // Configure PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPosition,
        this::setPose,
        () -> KINEMATICS.toChassisSpeeds(getModuleStates()),
        this::runSwerve,
        new HolonomicPathFollowerConfig(
            translationPathplannerConstants,
            rotationPathplannerConstants,
            MAX_LINEAR_SPEED_MPS,
            DRIVEBASE_RADIUS_M,
            new ReplanningConfig(true, true)),
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
    gyroIO.updateInputs(gyroIOInputs);
    for (var module : modules) {
      module.updateInputs();
    }
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
        discretize(speeds); // Compensate for translational skew when rotating
    SwerveModuleState[] setpointStates = KINEMATICS.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, MAX_LINEAR_SPEED_MPS); // Normalize speeds

    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    if (!modulesOrienting) {
      currentSetpoint =
          setpointGenerator.generateSetpoint(MODULE_LIMITS, currentSetpoint, discreteSpeeds, 0.02);

      // run modules
      for (int i = 0; i < modules.length; i++) {
        // Optimize setpoints
        optimizedSetpointStates[i] =
            SwerveModuleState.optimize(currentSetpoint.moduleStates()[i], modules[i].getAngle());
        optimizedSetpointStates[i] =
            (Math.abs(optimizedSetpointStates[i].speedMetersPerSecond / MAX_LINEAR_SPEED_MPS)
                    > 0.01)
                ? modules[i].setDesiredState(optimizedSetpointStates[i])
                : modules[i].setDesiredState(
                    new SwerveModuleState(
                        optimizedSetpointStates[i].speedMetersPerSecond, modules[i].getAngle()));

        modules[i].setDesiredState(optimizedSetpointStates[i]);
      }
    } else {
      for (int i = 0; i < 4; i++) {
        optimizedSetpointStates[i] =
            modules[i].setDesiredState(
                setpointStates[i]); // setDesiredState returns the optimized state
      }
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
    poseEstimator.resetPosition(getRotation(), getModulePositions(), pose);
    currentPose = pose;
    //gyroIO.resetGyro(currentPose.getRotation());
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
    return new SwerveDriveKinematics(getModuleTranslations());
  }

  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X_M / 2.0, TRACK_WIDTH_Y_M / 2.0),
      new Translation2d(TRACK_WIDTH_X_M / 2.0, -TRACK_WIDTH_Y_M / 2.0),
      new Translation2d(-TRACK_WIDTH_X_M / 2.0, TRACK_WIDTH_Y_M / 2.0),
      new Translation2d(-TRACK_WIDTH_X_M / 2.0, -TRACK_WIDTH_Y_M / 2.0)
    };
  }

  public ChassisSpeeds discretize(ChassisSpeeds speeds) {
    double dt = 0.02;
    var desiredDeltaPose =
        new Pose2d(
            speeds.vxMetersPerSecond * dt,
            speeds.vyMetersPerSecond * dt,
            new Rotation2d(speeds.omegaRadiansPerSecond * dt * 3));
    var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
  }
}
