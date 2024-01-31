package frc.robot.subsystems.shooter.LeadScrewArm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.ScrewArmController;
import frc.robot.util.ScrewArmKinematics;
import org.littletonrobotics.junction.Logger;

public class ScrewArmSim implements ScrewArmIO {
  private SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          1.0,
          0.005,
          ScrewArmConstants.kPivotLength,
          Math.toRadians(15),
          Math.toRadians(75),
          false,
          Math.toRadians(15),
          VecBuilder.fill(0.0));

  private Rotation2d screwArmAngle = new Rotation2d();
  private double appliedVolts = 0;

  private Mechanism2d mechanismField = new Mechanism2d(80, 80);
  private MechanismRoot2d armRoot = mechanismField.getRoot("ArmPivot", 0, 0);
  private final MechanismLigament2d armPivot =
      armRoot.append(
          new MechanismLigament2d(
              "pivotArm",
              ScrewArmConstants.kDriverLength,
              Math.toRadians(15),
              10,
              new Color8Bit(255, 0, 0)));
  private final MechanismLigament2d armDriver =
      armPivot.append(
          new MechanismLigament2d(
              "DrivenArm",
              ScrewArmConstants.kDriverLength,
              ScrewArmKinematics.getDrivenAngle(Rotation2d.fromDegrees(15)).getRadians(),
              10,
              new Color8Bit(0, 255, 0)));

  // Inverted for sake of PID to go the right direction
  private final ScrewArmController screwArmController =
      new ScrewArmController(() -> screwArmAngle, (s) -> setScrewArmVolts(s));

  @Override
  public void updateInputs(ScrewArmInputs inputs) {
    armSim.update(0.02);

    screwArmAngle = Rotation2d.fromRadians(armSim.getAngleRads());
    inputs.screwArmDegrees = screwArmAngle;
    inputs.screwArmDegreesGoal = screwArmController.getGoal();
    inputs.screwArmDegreesSetpoint = screwArmController.getSetpoint();
    inputs.screwArmAppliedVolts = appliedVolts;
    inputs.screwArmCurrentAmps = armSim.getCurrentDrawAmps();
    inputs.screwArmMotorTempC = 0;
    inputs.screwArmPositionMeters = ScrewArmKinematics.getLengthAlongScrew(screwArmAngle);
    inputs.screwArmVelocityMeters = 0;
    inputs.screwArmAtGoal = screwArmController.atGoal();
    inputs.screwArmAtSetpoint = screwArmController.atSetpoint();

    armPivot.setAngle(screwArmAngle);
    armDriver.setAngle(0); // ScrewArmKinematics.getDrivenAngle(screwArmAngle).div(-1));
    Logger.recordOutput("ScrewArm", mechanismField);
    // SmartDashboard.putData("ScrewArm", mechanismField);
  }

  @Override
  public void setScrewArmVolts(double volts) {
    appliedVolts =
        MathUtil.clamp(volts, -ScrewArmConstants.kMaxVoltage, ScrewArmConstants.kMaxVoltage);
    appliedVolts *= ScrewArmKinematics.getPerpendicularAngleDifference(screwArmAngle).getCos();
    armSim.setInputVoltage(appliedVolts);
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
}
