// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Robot;
import frc.robot.RobotStates.ShooterStates;
import frc.robot.subsystems.shooter.angler.AnglerIO;
import frc.robot.subsystems.shooter.angler.AnglerIOInputsAutoLogged;
import frc.robot.subsystems.shooter.launcher.LauncherIO;
import frc.robot.subsystems.shooter.launcher.LauncherIOInputsAutoLogged;
import frc.robot.utils.debugging.LoggedTunableNumber;
import frc.robot.utils.debugging.SysIDCharacterization;
import frc.robot.utils.math.LinearProfile;
import frc.robot.utils.math.ScrewArmFeedforward;
import java.util.HashMap;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Shooter subsystem */
public class Shooter extends SubsystemBase {
  private static LoggedTunableNumber angleTunableNumber = new LoggedTunableNumber("Shooter/AngleDebuggingDegrees", 0.0);
   
  public static enum AnglerSetpoints {
    AIM(() -> TargetingSystem.getInstance().getLaunchMapAngle()),
    CLIMB(() -> Rotation2d.fromDegrees(25.0)),
    INTAKE(() -> Rotation2d.fromDegrees(45.0)),
    IDLE(() -> anglerPosition),
    PODIUM(() -> Rotation2d.fromDegrees(34.5)),
    SPEAKER(() -> Rotation2d.fromDegrees(57)),
    AMP(() -> Rotation2d.fromDegrees(54.0)),
    FEEDER(() -> Rotation2d.fromDegrees(50.0)),
    DEBUGGING(() -> Rotation2d.fromDegrees(angleTunableNumber.get()));

    private Supplier<Rotation2d> angleSupplier;

    AnglerSetpoints(Supplier<Rotation2d> angle) {
      this.angleSupplier = angle;
    }

    public Supplier<Rotation2d> getAngle() {
      return angleSupplier;
    }
  }

  public static enum LauncherSetpoints {
    EJECT(() -> 5.0, () -> 5.0),
    IDLE(() -> 9.0, () -> 9.0),
    SPEAKER_SHOT(() -> 38.0, () -> 38.0),
    FULL_SPEED(() -> 42.0, () -> 42.0),
    FEEDER(() -> 30.0, () -> 30.0),
    // 12.5: 7 / 9
    // 12.65: 0
    // 12.58: 9/12
    AMP(() -> -0.1, () -> 12.54),
    OFF(() -> 0.0, () -> 0.0);

    private DoubleSupplier topSpeedSupplierMPS;
    private DoubleSupplier bottomSpeedSupplierMPS;

    LauncherSetpoints(DoubleSupplier topSpeedMPS, DoubleSupplier bottomSpeedMPS) {
      this.topSpeedSupplierMPS = topSpeedMPS;
      this.bottomSpeedSupplierMPS = bottomSpeedMPS;
    }

    public DoubleSupplier getTopSpeedMPS() {
      return topSpeedSupplierMPS;
    }

    public DoubleSupplier getBottomSpeedMPS() {
      return bottomSpeedSupplierMPS;
    }
  }

  public static Rotation2d anglerPosition = null;

  private AnglerIO anglerIO;
  private AnglerIOInputsAutoLogged anglerIOInputs = new AnglerIOInputsAutoLogged();
  private LauncherIO launcherIO;
  private LauncherIOInputsAutoLogged launcherIOInputs = new LauncherIOInputsAutoLogged();

  private final Rotation2d minAngle = Rotation2d.fromDegrees(26.5);
  private final Rotation2d maxAngle = Rotation2d.fromDegrees(57.0);

  // Default to real values
  private ProfiledPIDController anglerFeedback =
      new ProfiledPIDController(0.49, 2.0, 0.018, new TrapezoidProfile.Constraints(1000.0, 1000.0));
  private ScrewArmFeedforward anglerFeedforward = new ScrewArmFeedforward(0.2, 0.0);

  private LinearProfile topWheelProfile = new LinearProfile(50, 0.02);
  private LinearProfile bottomWheelProfile = new LinearProfile(50, 0.02);

  private LoggedTunableNumber anglerFeedbackP;
  private LoggedTunableNumber anglerFeedbackI;
  private LoggedTunableNumber anglerFeedbackD;
  private LoggedTunableNumber anglerFeedbackV;
  private LoggedTunableNumber anglerFeedbackA;

  private LoggedTunableNumber anglerFeedforwardU;
  private LoggedTunableNumber anglerFeedforwardL;

  private ShooterVisualizer anglerVisualizer = new ShooterVisualizer(new Rotation2d());

  private AnglerSetpoints anglerSetpoint = null;
  private LauncherSetpoints launcherSetpointMPS = null;

  private boolean angleEncoderCalibrated = false;
  private Rotation2d angleOffset = new Rotation2d();
  private Rotation2d currentAngle = new Rotation2d();

  public Shooter(AnglerIO anglerIO, LauncherIO launcherIO) {
    this.anglerIO = anglerIO;
    this.launcherIO = launcherIO;

    if (Constants.currentRobot == Robot.SYNTH) {
      switch (Constants.currentMode) {
        case REAL:
          anglerFeedback.setP(0.49);
          anglerFeedback.setI(2.0);
          anglerFeedback.setD(0.018);
          anglerFeedback.setConstraints(new TrapezoidProfile.Constraints(1000.0, 1000.0));

          anglerFeedforward.updateU(0.2);
          anglerFeedforward.updateL(0.0);
          break;
        case SIM:
          anglerFeedback.setP(0.1);
          anglerFeedback.setI(0.0);
          anglerFeedback.setD(0.5);
          anglerFeedback.setConstraints(new TrapezoidProfile.Constraints(50.0, 50.0));

          anglerFeedforward.updateU(0.0);
          anglerFeedforward.updateL(0.0);
          break;
        default:
          anglerFeedback.setP(0.0);
          anglerFeedback.setI(0.0);
          anglerFeedback.setD(0.0);
          anglerFeedback.setConstraints(new TrapezoidProfile.Constraints(0.0, 0.0));

          anglerFeedforward.updateU(0.0);
          anglerFeedforward.updateL(0.0);
          break;
      }
    }

    anglerFeedbackP = new LoggedTunableNumber("Shooter/Angler/Feedback/P", anglerFeedback.getP());
    anglerFeedbackI = new LoggedTunableNumber("Shooter/Angler/Feedback/I", anglerFeedback.getI());
    anglerFeedbackD = new LoggedTunableNumber("Shooter/Angler/Feedback/D", anglerFeedback.getD());
    anglerFeedbackV =
        new LoggedTunableNumber(
            "Shooter/Angler/Feedback/V", anglerFeedback.getConstraints().maxVelocity);
    anglerFeedbackA =
        new LoggedTunableNumber(
            "Shooter/Angler/Feedback/A", anglerFeedback.getConstraints().maxAcceleration);
    anglerFeedforwardU =
        new LoggedTunableNumber("Shooter/Angler/Feedforward/U", anglerFeedforward.getU());
    anglerFeedforwardL =
        new LoggedTunableNumber("Shooter/Angler/Feedforward/L", anglerFeedforward.getL());

    resetAnglerFeedback();
    anglerFeedback.setTolerance(0.25);
    anglerFeedback.setIZone(20.0);
    anglerFeedback.setIntegratorRange(-0.5, 0.5);
  }

  @Override
  public void periodic() {
    anglerIO.updateInputs(anglerIOInputs);
    Logger.processInputs("Shooter/Angler/Inputs", anglerIOInputs);
    launcherIO.updateInputs(launcherIOInputs);
    Logger.processInputs("Shooter/Launcher/Inputs", launcherIOInputs);

    if (DriverStation.isDisabled()) {
      stopMotors(true, true);
    }

    if (!angleEncoderCalibrated) {
      for (int i = 0; i < 100; i++) {
        if (anglerIOInputs.anglerDutyCycleFrequency == 955) {
          angleOffset =
              Rotation2d.fromRadians(
                      MathUtil.inputModulus(
                          anglerIOInputs.anglerAbsolutePosition.getRadians(), 0, 2.0 * Math.PI))
                  .minus(anglerIOInputs.anglerRelativePosition);
          angleEncoderCalibrated = true;
          break;
        }
        if (i == 99) {
          angleOffset =
              Rotation2d.fromRadians(
                      MathUtil.inputModulus(
                          anglerIOInputs.anglerAbsolutePosition.getRadians(), 0, 2.0 * Math.PI))
                  .minus(anglerIOInputs.anglerRelativePosition);
          angleEncoderCalibrated = true;
          break;
        }
        anglerVisualizer =
            new ShooterVisualizer(anglerIOInputs.anglerRelativePosition.plus(angleOffset));
      }
      currentAngle = anglerIOInputs.anglerRelativePosition.plus(angleOffset);

      resetAnglerFeedback();
    }

    currentAngle = anglerIOInputs.anglerRelativePosition.plus(angleOffset);

    if (anglerSetpoint != null) {
      Rotation2d angleSetpoint =
          Rotation2d.fromDegrees(
              MathUtil.clamp(
                  anglerSetpoint.getAngle().get().getDegrees(),
                  minAngle.getDegrees(),
                  maxAngle.getDegrees()));

      double anglerFeedbackOutput =
          anglerFeedback.calculate(currentAngle.getDegrees(), angleSetpoint.getDegrees());
      double anglerFeedforwardOutput = anglerFeedforward.calculate(currentAngle, angleSetpoint);

      double anglerCombinedOutput = (anglerFeedbackOutput + anglerFeedforwardOutput);

      anglerIO.setVolts(anglerCombinedOutput);

      Logger.recordOutput("Shooter/Angler/Feedback/Output", anglerFeedbackOutput);
      Logger.recordOutput("Shooter/Angler/Feedforward/Output", anglerFeedforwardOutput);
      Logger.recordOutput("Shooter/Angler/CombinedOutput", anglerCombinedOutput);
    }

    if (launcherSetpointMPS != null) {
      // System.out.println("HAHA");
      launcherIO.setTopVelocity(
          topWheelProfile.calculateSetpoint(), topWheelProfile.getCurrentAcceleration());
      launcherIO.setBottomVelocity(
          bottomWheelProfile.calculateSetpoint(), bottomWheelProfile.getCurrentAcceleration());
    }

    anglerVisualizer.updateShooterAngle(currentAngle);

    if (Constants.tuningMode) {
      updateTunableNumbers();
    }
  }

  private void updateTunableNumbers() {
    if (anglerFeedbackP.hasChanged(hashCode())
        || anglerFeedbackI.hasChanged(hashCode())
        || anglerFeedbackD.hasChanged(hashCode())
        || anglerFeedbackV.hasChanged(hashCode())
        || anglerFeedbackA.hasChanged(hashCode())) {
      anglerFeedback.setP(anglerFeedbackP.get());
      anglerFeedback.setI(anglerFeedbackI.get());
      anglerFeedback.setD(anglerFeedbackD.get());

      anglerFeedback.setConstraints(
          new TrapezoidProfile.Constraints(anglerFeedbackV.get(), anglerFeedbackA.get()));
    }
    if (anglerFeedforwardU.hasChanged(hashCode()) || anglerFeedforwardL.hasChanged(hashCode())) {
      anglerFeedforward.updateU(anglerFeedforwardU.get());
      anglerFeedforward.updateL(anglerFeedforwardL.get());
    }
  }

  public HashMap<ShooterStates, Command> mapToCommand() {
    HashMap<ShooterStates, Command> shooterCommandMap = new HashMap<>();
    shooterCommandMap.put(ShooterStates.OFF, Commands.runOnce(() -> stopMotors(true, true), this));
    shooterCommandMap.put(
        ShooterStates.AIM,
        setShooterState(AnglerSetpoints.AIM, LauncherSetpoints.SPEAKER_SHOT));
    shooterCommandMap.put(
        ShooterStates.INTAKE, setShooterState(AnglerSetpoints.INTAKE, LauncherSetpoints.OFF));
    shooterCommandMap.put(
        ShooterStates.CLIMB, setShooterState(AnglerSetpoints.CLIMB, LauncherSetpoints.OFF));
    shooterCommandMap.put(
        ShooterStates.EJECT, setShooterState(AnglerSetpoints.CLIMB, LauncherSetpoints.EJECT));
    shooterCommandMap.put(ShooterStates.UP, setAnglerManual(5.0));
    shooterCommandMap.put(ShooterStates.DOWN, setAnglerManual(-5.0));
    shooterCommandMap.put(
        ShooterStates.FIRE,
        Commands.runOnce(() -> setLauncherVelocityMPS(LauncherSetpoints.SPEAKER_SHOT), this));
    shooterCommandMap.put(
        ShooterStates.IDLE,
        Commands.runOnce(
            () -> {
              anglerPosition = currentAngle;
              setMotors(null, LauncherSetpoints.OFF);
              setAnglerVolts(0.0);
            },
            this));
    shooterCommandMap.put(
        ShooterStates.PODIUM,
        setShooterState(AnglerSetpoints.PODIUM, LauncherSetpoints.SPEAKER_SHOT));
    shooterCommandMap.put(
        ShooterStates.SPEAKER,
        setShooterState(AnglerSetpoints.SPEAKER, LauncherSetpoints.SPEAKER_SHOT));
    shooterCommandMap.put(
        ShooterStates.FEEDER,
        setShooterState(AnglerSetpoints.FEEDER, LauncherSetpoints.FULL_SPEED));
    shooterCommandMap.put(
        ShooterStates.REV_AMP, setShooterState(AnglerSetpoints.AMP, LauncherSetpoints.AMP));

    shooterCommandMap.put(
      ShooterStates.SHOOT_AMP, Commands.runOnce(() -> {
        setMotors(anglerSetpoint, null);
        launcherIO.setTopVolts(0);
      }, this));

    return shooterCommandMap;
  }

  public Command setShooterState(
      AnglerSetpoints anglerState, LauncherSetpoints launcherState) {
    return new InstantCommand(() -> setMotors(anglerState, launcherState), this);
  }

  public Command setAnglerManual(double volts) {
    return Commands.runOnce(
        () -> {
          anglerSetpoint = null;
          setAnglerVolts(volts);
        },
        this);
  }

  public void setMotors(AnglerSetpoints anglerGoal, LauncherSetpoints launcherGoal) {
    setAnglerPosition(anglerGoal);
    setLauncherVelocityMPS(launcherGoal);
  }

  public void setAnglerPosition(AnglerSetpoints position) {
    anglerSetpoint = position;

    if (anglerSetpoint != null) {
      DoubleSupplier errorDegrees =
          () -> Math.abs(anglerSetpoint.getAngle().get().minus(currentAngle).getDegrees());
      Logger.recordOutput("Shooter/ErrorDegrees", errorDegrees.getAsDouble());

      if (anglerSetpoint != null && errorDegrees.getAsDouble() > 2.0) {
        resetAnglerFeedback();
      }
    }
  }

  public void resetAnglerFeedback() {
    anglerFeedback.reset(currentAngle.getDegrees(), 0.0);
  }

  public void setLauncherVelocityMPS(LauncherSetpoints velocityMPS) {
    launcherSetpointMPS = velocityMPS;
    if(launcherSetpointMPS != null) {
      topWheelProfile.setGoal(launcherSetpointMPS.topSpeedSupplierMPS.getAsDouble(), launcherIOInputs.topFlywheelVelocityMPS);
      bottomWheelProfile.setGoal(launcherSetpointMPS.bottomSpeedSupplierMPS.getAsDouble(), launcherIOInputs.bottomFlywheelVelocityMPS);
    }
  }

  public void setAnglerVolts(double volts) {
    anglerIO.setVolts(volts);
  }

  public void setLauncherVolts(double topFlywheelVolts, double bottomFlywheelVolts) {
    launcherIO.setTopVolts(topFlywheelVolts);
    launcherIO.setBottomVolts(bottomFlywheelVolts);
  }

  public void stopMotors(boolean stopAngler, boolean stopLaunchers) {
    if (stopAngler) {
      anglerSetpoint = null;
      anglerIO.setVolts(0.0);
    }
    if (stopLaunchers) {
      launcherSetpointMPS = null;
      launcherIO.setTopVolts(0.0);
      launcherIO.setBottomVolts(0.0);
    }
  }

  @AutoLogOutput(key = "Shooter/Angler/Position")
  public Rotation2d getAnglerPosition() {
    return currentAngle;
  }

  @AutoLogOutput(key = "Shooter/Angler/PositionDegrees")
  public double getAnglerDegrees() {
    return currentAngle.getDegrees();
  }

  @AutoLogOutput(key = "Shooter/Angler/AtAllSetpoints")
  public boolean atAllSetpoint() {
    return getAnglerPositionError() < 0.6
        && launcherIOInputs.topFlywheelErrorMPS < 1
        && launcherIOInputs.bottomFlywheelErrorMPS < 1;
  }

  @AutoLogOutput(key = "Shooter/Angler/Feedback/SetpointPosition")
  public Rotation2d getAnglerSetpointPosition() {
    return Rotation2d.fromDegrees(anglerFeedback.getSetpoint().position);
  }

  @AutoLogOutput(key = "Shooter/Angler/Feedback/SetpointVelocity")
  public double getAnglerSetpointVelocity() {
    return anglerFeedback.getSetpoint().velocity;
  }

  @AutoLogOutput(key = "Shooter/Angler/Feedback/GoalPosition")
  public double getAnglerGoalPosition() {
    return anglerFeedback.getGoal().position;
  }

  @AutoLogOutput(key = "Shooter/Angler/Feedback/PositionError")
  public double getAnglerPositionError() {
    return anglerFeedback.getPositionError();
  }

  @AutoLogOutput(key = "Shooter/Angler/Feedback/VelocityError")
  public double getAnglerVelocityError() {
    return anglerFeedback.getVelocityError();
  }

  @AutoLogOutput(key = "Shooter/Angler/Feedback/AtGoal")
  public boolean isAnglerAtGoal() {
    return anglerFeedback.atGoal();
  }

  public double getTopLauncherError() {
    return launcherIOInputs.topFlywheelErrorMPS;
  }

  public double getBottomLauncherError() {
    return launcherIOInputs.bottomFlywheelErrorMPS;
  }

  public Command characterizeFlywheel() {
    return SysIDCharacterization.runShooterSysIDTests(
        (volts) -> {
          launcherSetpointMPS = null;
          launcherIO.setTopVolts(volts);
          launcherIO.setBottomVolts(volts);
        },
        this);
  }
}
