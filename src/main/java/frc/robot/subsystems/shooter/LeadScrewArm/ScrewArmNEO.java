package frc.robot.subsystems.shooter.LeadScrewArm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.util.ScrewArmKinematics;

public class ScrewArmNEO implements ScrewArmIO{
    public CANSparkMax screwArmMotor;
    public RelativeEncoder screwArmRodEncoder;
    public DutyCycleEncoder screwArmPivotEncoder;

    public ProfiledPIDController screwArmPIDController;
    public Rotation2d screwArmSetpointAngle = new Rotation2d();
    public Rotation2d screwArmAngle = new Rotation2d();

    public ScrewArmNEO() {
        configScrewArmMotor();
        this.screwArmPivotEncoder = new DutyCycleEncoder(1);

        screwArmPIDController = new ProfiledPIDController(
            0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
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
    }

    @Override
    public void setScrewArmVolts(double volts) {
        screwArmMotor.setVoltage(volts);
    }

    @Override
    public void setGoal(Rotation2d goal) {
        screwArmPIDController.setGoal( goal.getDegrees() );
    }

    @Override
    public void initPID(Rotation2d measure) {
        screwArmPIDController.reset( screwArmAngle.getDegrees() );
    }

    @Override
    public void executePID(Rotation2d goal) {
        double PID = 
            screwArmPIDController.calculate( screwArmAngle.getDegrees() ) 
            * ScrewArmKinematics.scaleVoltage(
                Rotation2d.fromDegrees(
                    screwArmPIDController.getSetpoint().position ) );

        double FF = 
            Math.signum(screwArmPIDController.getSetpoint().velocity) * ScrewArmConstants.kScrewArmKS
            + ScrewArmConstants.kScrewArmKG * ScrewArmKinematics.getGravityVector(
                Rotation2d.fromDegrees( screwArmPIDController.getSetpoint().position ) );

        setScrewArmVolts(PID + FF);
    }

    public void configScrewArmMotor() {
        screwArmMotor = new CANSparkMax(1, MotorType.kBrushless);
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