package frc.robot.subsystems.shooter.Angler;

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
import org.littletonrobotics.junction.Logger;

public class AnglerSim implements AnglerIO {
  private SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          50.0,
          AnglerConstants.kJKGMetersSquared,
          AnglerConstants.kPivotLengthMeters,
          Math.toRadians(15),
          Math.toRadians(75),
          false,
          Math.toRadians(15),
          VecBuilder.fill(0.001));

  private Rotation2d anglerAngle = new Rotation2d();
  private double appliedVolts = 0;

  private Mechanism2d mechanismField = new Mechanism2d(4, 4);
  private MechanismRoot2d armRoot = mechanismField.getRoot("ArmPivot", 0, 0);
  private final MechanismLigament2d armPivot =
      armRoot.append(
          new MechanismLigament2d(
              "pivotArm",
              AnglerConstants.kPivotLengthMeters,
              Math.toRadians(15),
              10,
              new Color8Bit(255, 0, 0)));

  // Inverted for sake of PID to go the right direction
  private final AnglerController anglerController =
      new AnglerController(() -> anglerAngle, this::setAnglerVolts);

  @Override
  public void updateInputs(AnglerInputs inputs) {
    armSim.update(0.02);

    anglerAngle = Rotation2d.fromRadians(armSim.getAngleRads());
    inputs.anglerAngle = anglerAngle;
    inputs.anglerAngleGoal = anglerController.getGoal();
    inputs.anglerAngleSetpoint = anglerController.getSetpoint();
    inputs.anglerAppliedVolts = appliedVolts;
    inputs.anglerCurrentAmps = armSim.getCurrentDrawAmps();
    inputs.anglerMotorTempC = 0;
    inputs.anglerAtGoal = anglerController.atGoal();
    inputs.anglerAtSetpoint = anglerController.atSetpoint();

    armPivot.setAngle(anglerAngle);
    Logger.recordOutput("angler", mechanismField);
  }

  @Override
  public void setAnglerVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, AnglerConstants.kMinVoltage, AnglerConstants.kMaxVoltage);
    armSim.setInputVoltage(appliedVolts);
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
}
