package frc.robot.subsystems.shooter.LeadScrewArm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import frc.robot.util.ScrewArmController;

public class ScrewArmNEO implements ScrewArmIO {
  public CANSparkMax screwArmMotor;
  public RelativeEncoder screwArmRodEncoder;
  public DutyCycleEncoder screwArmPivotEncoder;

  public Rotation2d screwArmSetpointAngle = new Rotation2d();
  public Rotation2d screwArmAngle = new Rotation2d();

  public ScrewArmController screwArmController;

  public ScrewArmNEO(int id) {
    configScrewArmMotor(id);
    this.screwArmPivotEncoder = new DutyCycleEncoder(ScrewArmConstants.kEncoderID);

    screwArmController =
        new ScrewArmController(
            () -> screwArmAngle,
            this::setScrewArmVolts);
  }

  @Override
  public void updateInputs(ScrewArmInputs inputs) {
    screwArmAngle = Rotation2d.fromRotations(screwArmPivotEncoder.getAbsolutePosition());
    inputs.screwArmDegrees = screwArmAngle;
    inputs.screwArmDegreesSetpoint = screwArmSetpointAngle;
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
    screwArmMotor.setVoltage(volts);
  }

  @Override
  public void setGoal(Rotation2d goal) {
    screwArmSetpointAngle = goal;
    screwArmController.setGoal( screwArmSetpointAngle );
  }

  @Override
  public void initPID() {
    screwArmController.reset( new TrapezoidProfile.State( screwArmAngle.getDegrees(), 0 ) );
  }

  @Override
  public void executePID() {
    screwArmController.executePIDClamped( screwArmSetpointAngle );
  }

  @Override
  public boolean atGoal() {
    return screwArmController.atGoal();
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