package frc.robot.subsystems.shooter.angler;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

public class AnglerNEO implements AnglerIOi {
  public CANSparkMax anglerMotor;
  public DutyCycleEncoder anglerPivotAbsoluteEncoder;
  public Encoder anglerPivotRelativeEncoder;

  public Rotation2d anglerAngle = new Rotation2d();
  public Rotation2d anglerOffset = new Rotation2d();

  public AnglerController anglerController;

  public AnglerNEO(int id) {
    configanglerMotor(id);
    this.anglerPivotAbsoluteEncoder = new DutyCycleEncoder(AnglerConstants.kAbsoluteEncoderID);
    this.anglerPivotAbsoluteEncoder.setDistancePerRotation(360);
    this.anglerAngle = Rotation2d.fromDegrees(anglerPivotAbsoluteEncoder.getAbsolutePosition());

    this.anglerPivotRelativeEncoder =
        new Encoder(AnglerConstants.kRelativeEncoderAID, AnglerConstants.kRelativeEncoderBID);
    this.anglerPivotRelativeEncoder.setDistancePerPulse(360 / 8192);

    anglerController = new AnglerController(() -> anglerAngle, this::setAnglerVolts);
  }

  @Override
  public void updateInputs(AnglerIOInputs inputs) {
    anglerAngle = Rotation2d.fromRotations(anglerPivotAbsoluteEncoder.getAbsolutePosition());
    inputs.anglerAngle = anglerAngle;
    inputs.anglerAngleGoal = anglerController.getGoal();
    inputs.anglerAngleSetpoint = anglerController.getSetpoint();
    inputs.anglerAppliedVolts = anglerMotor.getBusVoltage();
    inputs.anglerCurrentAmps = anglerMotor.getOutputCurrent();
    inputs.anglerMotorTempC = anglerMotor.getMotorTemperature();
    inputs.anglerAtGoal = anglerController.atGoal();
    inputs.anglerAtSetpoint = anglerController.atSetpoint();
    anglerController.updateTunablePID();
  }

  @Override
  public void setAnglerVolts(double volts) {
    anglerMotor.setVoltage(
        MathUtil.clamp(volts, AnglerConstants.kMinVoltage, AnglerConstants.kMaxVoltage));
  }

  @Override
  public void setGoal(Rotation2d goal) {
    anglerController.setGoal(goal);
  }

  @Override
  public void initPID() {
    anglerController.reset(new TrapezoidProfile.State(anglerAngle.getDegrees(), 0));
  }

  @Override
  public void executePID() {
    anglerController.executePIDClamped();
  }

  public void configanglerMotor(int id) {
    anglerMotor = new CANSparkMax(id, MotorType.kBrushless);
    anglerMotor.restoreFactoryDefaults();
    anglerMotor.clearFaults();
    anglerMotor.setSmartCurrentLimit(40);
    anglerMotor.setSecondaryCurrentLimit(40);
    anglerMotor.setInverted(false);
    anglerMotor.enableVoltageCompensation(12);
    anglerMotor.setIdleMode(IdleMode.kBrake);

    anglerMotor.burnFlash();
  }
}
