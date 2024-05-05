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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.utils.debugging.SysIDCharacterization;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Swerve drive */
public class Drive extends SubsystemBase {
  public static final double TRACK_WIDTH_X_M = Units.inchesToMeters(24.25);
  public static final double TRACK_WIDTH_Y_M = Units.inchesToMeters(24.25);
  public static final double DRIVEBASE_RADIUS_M =
      Math.hypot(TRACK_WIDTH_X_M / 2.0, TRACK_WIDTH_Y_M / 2.0);
  public static final double MAX_LINEAR_SPEED_MPS = 4.8;
  public static final double MAX_ANGULAR_SPEED_MPS = MAX_LINEAR_SPEED_MPS / DRIVEBASE_RADIUS_M;
  // Second argument is the max accel
  public static final ModuleLimits MODULE_LIMITS =
      new ModuleLimits(MAX_LINEAR_SPEED_MPS, MAX_LINEAR_SPEED_MPS * 5, 12 * 2 * Math.PI);

  private final Translation2d[] MODULE_TRANSLATIONS = getModuleTranslations();
  private final SwerveDriveKinematics KINEMATICS = getKinematics();
  private ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();

  private SwerveSetpoint currentSetpoint =
      new SwerveSetpoint(
          new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
          });
  private SwerveSetpointGenerator setpointGenerator =
      new SwerveSetpointGenerator(KINEMATICS, MODULE_TRANSLATIONS);
  private boolean areModulesOrienting = false;

  private GyroIO gyroIO;
  private GyroIOInputsAutoLogged gyroIOInputs = new GyroIOInputsAutoLogged();

  private Module[] modules = new Module[4]; // FL FR BL BR

  private Pose2d currentPose = new Pose2d();
  private Pose2d filteredPose = new Pose2d();

  // Used to compare pose estimator and odometry
  private SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(KINEMATICS, getRotation(), getModulePositions());

  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(KINEMATICS, getRotation(), getModulePositions(), currentPose);

  private PIDConstants translationPathplannerConstants = new PIDConstants(1.25, 0.0, 0.0);
  private PIDConstants rotationPathplannerConstants = new PIDConstants(1.75, 0.0, 0.0);
  private boolean PProtationTargetOverride = false;

  private LinearFilter xFilter = LinearFilter.movingAverage(5);
  private LinearFilter yFilter = LinearFilter.movingAverage(5);

  private Field2d field = new Field2d();

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

    AutoBuilder.configureHolonomic(
        () -> {
          if(Constants.currentMode == Mode.REAL) {
            return getPoseEstimate();
          } else {
            return getOdometryPose();
          }
        },
        this::setPose,
        () -> KINEMATICS.toChassisSpeeds(getModuleStates()),
        this::runSwerve,
        new HolonomicPathFollowerConfig(
            translationPathplannerConstants,
            rotationPathplannerConstants,
            MAX_LINEAR_SPEED_MPS,
            DRIVEBASE_RADIUS_M,
            new ReplanningConfig(true, false)),
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

    SmartDashboard.putData(field);
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroIOInputs);
    for (var module : modules) {
      module.updateInputs();
      Logger.processInputs("Drive/Gyro/Inputs", gyroIOInputs);

      module.periodic();

      if (DriverStation.isDisabled()) {
        module.stop();
        // Log empty states
        Logger.recordOutput("Drive/Swerve/Setpoints", new SwerveModuleState[] {});
        Logger.recordOutput("Drive/Swerve/SetpointsOptimized", new SwerveModuleState[] {});
      }

      if (DriverStation.isEStopped()) {
        module.stop();
        module.setBrake(true); // Apply brakes if E-Stopped
      }
    }

    if (gyroIOInputs.connected) {
      poseEstimator.update(getRotation(), getModulePositions());
      odometry.update(getRotation(), getModulePositions());
    } else {
      poseEstimator.update(
          Rotation2d.fromDegrees(
              (poseEstimator.getEstimatedPosition().getRotation().getDegrees()
                  + Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond) * 0.02)
              % 360.0),
          getModulePositions());
      odometry.update(
          Rotation2d.fromDegrees(
              (odometry.getPoseMeters().getRotation().getDegrees()
                  + Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond) * 0.02)
              % 360.0),
          getModulePositions());
    }

    currentPose = poseEstimator.getEstimatedPosition();

    filteredPose =
        new Pose2d(
            xFilter.calculate(getPoseEstimate().getX()),
            yFilter.calculate(getPoseEstimate().getY()),
            getPoseEstimate().getRotation());

    field.setRobotPose(getFilteredPose());
  }

  public void runSwerve(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = discretize(speeds); // Translational skew compensation
    desiredChassisSpeeds = discreteSpeeds;
    SwerveModuleState[] setpointStates = KINEMATICS.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, MAX_LINEAR_SPEED_MPS); // Normalize speeds

    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];

    if (!areModulesOrienting) {
      currentSetpoint =
          setpointGenerator.generateSetpoint(MODULE_LIMITS, currentSetpoint, discreteSpeeds, 0.02);

      // There is no joystick filtering because the setpoint generator handles that
      for (int i = 0; i < 4; i++) {
        optimizedSetpointStates[i] =
            SwerveModuleState.optimize(currentSetpoint.moduleStates()[i], modules[i].getAngle());

        modules[i].setDesiredState(optimizedSetpointStates[i]);
      }
    } else {
      for (int i = 0; i < 4; i++) {
        optimizedSetpointStates[i] =
            modules[i].setDesiredState(
                setpointStates[i]); // setDesiredState returns the optimized state
      }
    }

    Logger.recordOutput("Drive/Swerve/Setpoints", setpointStates);
    Logger.recordOutput("Drive/Swerve/SetpointsOptimized", optimizedSetpointStates);
  }

  private ChassisSpeeds discretize(ChassisSpeeds speeds) {
    double dt = 0.02;
    var desiredDeltaPose =
        new Pose2d(
            speeds.vxMetersPerSecond * dt,
            speeds.vyMetersPerSecond * dt,
            new Rotation2d(speeds.omegaRadiansPerSecond * dt * 3));
    var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
  }

  public void stop() {
    runSwerve(new ChassisSpeeds());
  }

  public void resetPose() {
    setPose(currentPose);
  }

  public void resetGyro() {
    gyroIO.resetGyro();
    setPoses(
      new Pose2d(currentPose.getTranslation(), getRotation()),
      new Pose2d(odometry.getPoseMeters().getTranslation(), getRotation()));
  }

  /** Set the pose of the robot */
  public void setPose(Pose2d pose) {
    setPoses(pose, pose);
  }

  public void setPoses(Pose2d visionPose, Pose2d odometryPose) {
    if (Constants.currentMode == Mode.SIM) {
      poseEstimator.resetPosition(visionPose.getRotation(), getModulePositions(), visionPose);
      odometry.resetPosition(odometryPose.getRotation(), getModulePositions(), odometryPose);
    } else {
      poseEstimator.resetPosition(getRotation(), getModulePositions(), visionPose);
      odometry.resetPosition(getRotation(), getModulePositions(), odometryPose);
    }

    currentPose = poseEstimator.getEstimatedPosition();
  }

  public void addVisionMeasurement(
      Pose2d visionMeasurement, double timestampS, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampS, stdDevs);
  }

  public PathConstraints getPathConstraints() {
    return new PathConstraints(
        3.0, 3.0, 
        Units.degreesToRadians(540.0), Units.degreesToRadians(720.0));
  }

  @AutoLogOutput(key = "Drive/Swerve/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getModuleState();
    }

    return states;
  }

  @AutoLogOutput(key = "Drive/Swerve/ModulePositions")
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] =
          modules[i] != null ? modules[i].getModulePosition() : new SwerveModulePosition();
    }

    return positions;
  }

  @AutoLogOutput(key = "Drive/Odometry/PoseEstimate")
  public Pose2d getPoseEstimate() {
    return currentPose;
  }

  @AutoLogOutput(key = "Drive/Odometry/DrivePose")
  public Pose2d getOdometryPose() {
    return odometry.getPoseMeters();
  }

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

  public ChassisSpeeds getChassisSpeeds() {
    return KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public ChassisSpeeds getDesiredChassisSpeeds() {
    return desiredChassisSpeeds;
  }

  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X_M / 2.0, TRACK_WIDTH_Y_M / 2.0),
      new Translation2d(TRACK_WIDTH_X_M / 2.0, -TRACK_WIDTH_Y_M / 2.0),
      new Translation2d(-TRACK_WIDTH_X_M / 2.0, TRACK_WIDTH_Y_M / 2.0),
      new Translation2d(-TRACK_WIDTH_X_M / 2.0, -TRACK_WIDTH_Y_M / 2.0)
    };
  }

  public SwerveDriveKinematics getKinematics() {
    return new SwerveDriveKinematics(getModuleTranslations());
  }

  @AutoLogOutput(key = "Drive/PP/RotationTargetOverride")
  public boolean getPPRotationTargetOverride() {
    return PProtationTargetOverride;
  }

  public void setPProtationTargetOverride(boolean override) {
    PProtationTargetOverride = override;
  }

  @AutoLogOutput(key = "Drive/Odometry/FilteredPose")
  public Pose2d updateFilteredPose() {
    filteredPose =
        new Pose2d(
            xFilter.calculate(getPoseEstimate().getX()),
            yFilter.calculate(getPoseEstimate().getY()),
            getPoseEstimate().getRotation());

    return filteredPose;
  }

  public Pose2d getFilteredPose() {
    return filteredPose;
  }

  public void resetModules() {
    for (int i = 0; i < 4; i++) {
      modules[i].reset();
    }
  }

  public Command characterizeDriveMotors() {
    return SysIDCharacterization.runDriveSysIDTests(
        (voltage) -> {
          for (var module : modules) {
            module.angleSetpoint = new Rotation2d(0.0);
            module.velocitySetpoint = null;
            module.setDriveVoltage(voltage);
          }
        },
        this);
  }
}
