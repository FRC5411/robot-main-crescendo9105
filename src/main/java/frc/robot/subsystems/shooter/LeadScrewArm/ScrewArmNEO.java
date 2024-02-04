package frc.robot.subsystems.shooter.LeadScrewArm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.util.ScrewArmController;

public class ScrewArmNEO implements ScrewArmIO {
  public CANSparkMax screwArmMotor;
  public RelativeEncoder screwArmRodEncoder;
  public DutyCycleEncoder screwArmPivotAbsoluteEncoder;
  // Potential future change to account for absolute encoder being slower
  // public Encoder screwArmPivotRelativeEncoder;

  public Rotation2d screwArmAngle = new Rotation2d();

  public ScrewArmController screwArmController;

  public ScrewArmNEO(int id) {
    configScrewArmMotor(id);
    this.screwArmPivotAbsoluteEncoder = new DutyCycleEncoder(ScrewArmConstants.kAbsoluteEncoderID);
    this.screwArmPivotAbsoluteEncoder.setDistancePerRotation(360);

    // this.screwArmPivotRelativeEncoder = new Encoder(
    //   ScrewArmConstants.kRelativeEncoderAID, ScrewArmConstants.kRelativeEncoderBID);
    // this.screwArmPivotRelativeEncoder.setDistancePerPulse(360 / 8192);

    screwArmController = new ScrewArmController(() -> screwArmAngle, this::setScrewArmVolts);
  }

  @Override
  public void updateInputs(ScrewArmInputs inputs) {
    screwArmAngle = Rotation2d.fromRotations(screwArmPivotAbsoluteEncoder.getDistance());
    inputs.screwArmAngle = screwArmAngle;
    inputs.screwArmAngleGoal = screwArmController.getGoal();
    inputs.screwArmAngleSetpoint = screwArmController.getSetpoint();
    inputs.screwArmAppliedVolts = screwArmMotor.getBusVoltage();
    inputs.screwArmCurrentAmps = screwArmMotor.getOutputCurrent();
    inputs.screwArmMotorTempC = screwArmMotor.getMotorTemperature();
    inputs.screwArmPositionMeters = screwArmRodEncoder.getPosition();
    inputs.screwArmVelocityMeters = screwArmRodEncoder.getVelocity();
    inputs.screwArmAtGoal = screwArmController.atGoal();
    inputs.screwArmAtSetpoint = screwArmController.atSetpoint();
  }

  @Override
  public void setScrewArmVolts(double volts) {
    screwArmMotor.setVoltage(
        MathUtil.clamp(volts, ScrewArmConstants.kMinVoltage, ScrewArmConstants.kMaxVoltage));
  }

  @Override
  public void setGoal(Rotation2d goal) {
    screwArmController.setGoal(goal);
  }

  @Override
  public void initPID() {
    screwArmController.reset(new TrapezoidProfile.State(screwArmAngle.getDegrees(), 0));
  }

  @Override
  public void executePID() {
    screwArmController.executePIDClamped();
  }

  public void configScrewArmMotor(int id) {
    screwArmMotor = new CANSparkMax(id, MotorType.kBrushless);
    screwArmMotor.restoreFactoryDefaults();
    screwArmMotor.clearFaults();
    screwArmMotor.setSmartCurrentLimit(40);
    screwArmMotor.setSecondaryCurrentLimit(40);
    screwArmMotor.setInverted(false);
    screwArmMotor.enableVoltageCompensation(12);
    screwArmMotor.setIdleMode(IdleMode.kBrake);

    screwArmRodEncoder = screwArmMotor.getEncoder();
    screwArmRodEncoder.setPositionConversionFactor(360);
    screwArmRodEncoder.setVelocityConversionFactor(360 / 60);
    screwArmMotor.burnFlash();
  }
}
