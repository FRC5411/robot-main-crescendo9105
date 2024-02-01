package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.shooter.LeadScrewArm.ScrewArmConstants;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ScrewArmController {
  public final Supplier<Rotation2d> measureSupplier;
  public final Consumer<Double> voltageConsumer;
  public final ProfiledPIDController controller;

  public ScrewArmController(
      Supplier<Rotation2d> measureSupplier, Consumer<Double> voltageConsumer) {
    this.measureSupplier = measureSupplier;
    this.voltageConsumer = voltageConsumer;
    this.controller =
        (RobotBase.isReal())
            ? new ProfiledPIDController(
                ScrewArmConstants.kP,
                ScrewArmConstants.kI,
                ScrewArmConstants.kD,
                ScrewArmConstants.kConstraints)
            : new ProfiledPIDController(
                ScrewArmConstants.kSimP,
                ScrewArmConstants.kSimI,
                ScrewArmConstants.kSimD,
                ScrewArmConstants.kSimConstraints);

    this.controller.enableContinuousInput(0, 360);
  }

  public ScrewArmController(
      Supplier<Rotation2d> measureSupplier,
      Consumer<Double> voltageConsumer,
      ProfiledPIDController controller) {
    this.measureSupplier = measureSupplier;
    this.voltageConsumer = voltageConsumer;
    this.controller = controller;
  }

  public void setGoal(Rotation2d goal) {
    controller.setGoal(goal.getDegrees());
  }

  public void reset(TrapezoidProfile.State state) {
    controller.reset(state);
  }

  public void executePIDClamped() {
    double PID =
        controller.calculate(measureSupplier.get().getDegrees())
            * ScrewArmKinematics.scaleVoltage(
                Rotation2d.fromDegrees(controller.getSetpoint().position));
    Logger.recordOutput("PID OUTPUT", PID);

    double FF =
        Math.signum(controller.getSetpoint().velocity) * ScrewArmConstants.kS
            // kG will always be 0 because of the gas shocks
            + ScrewArmConstants.kG
                * ScrewArmKinematics.getGravityUnitVector(
                    Rotation2d.fromDegrees(controller.getSetpoint().position));
    Logger.recordOutput("FF OUTPUT", FF);

    voltageConsumer.accept(
        MathUtil.clamp(PID, -ScrewArmConstants.kMaxVoltage, ScrewArmConstants.kMaxVoltage));
  }

  public Rotation2d getGoal() {
    return Rotation2d.fromDegrees(controller.getGoal().position);
  }

  public Rotation2d getSetpoint() {
    return Rotation2d.fromDegrees(controller.getSetpoint().position);
  }

  public boolean atGoal() {
    return controller.atGoal();
  }

  public boolean atSetpoint() {
    return controller.atSetpoint();
  }
}
