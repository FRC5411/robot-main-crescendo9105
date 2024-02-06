package frc.robot.subsystems.shooter.Angler;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class AnglerController {
  public final Supplier<Rotation2d> measureSupplier;
  public final Consumer<Double> voltageConsumer;
  public final ProfiledPIDController controller;
  public final ArmFeedforward armFF;

  private LoggedTunableNumber armControlP;
  private LoggedTunableNumber armControlI;
  private LoggedTunableNumber armControlD;

  public AnglerController(Supplier<Rotation2d> measureSupplier, Consumer<Double> voltageConsumer) {
    this.measureSupplier = measureSupplier;
    this.voltageConsumer = voltageConsumer;
    this.controller =
        (RobotBase.isReal())
            ? new ProfiledPIDController(
                AnglerConstants.kP,
                AnglerConstants.kI,
                AnglerConstants.kD,
                AnglerConstants.kConstraints)
            : new ProfiledPIDController(
                AnglerConstants.kSimP,
                AnglerConstants.kSimI,
                AnglerConstants.kSimD,
                AnglerConstants.kSimConstraints);
    this.armFF =
        (RobotBase.isReal())
            ? new ArmFeedforward(
                AnglerConstants.kS, AnglerConstants.kG, AnglerConstants.kV, AnglerConstants.kA)
            : new ArmFeedforward(
                AnglerConstants.kSimS,
                AnglerConstants.kSimG,
                AnglerConstants.kSimV,
                AnglerConstants.kSimA);

    this.controller.enableContinuousInput(0, 360);

    this.armControlP = new LoggedTunableNumber("Angler/P", controller.getP());
    this.armControlI = new LoggedTunableNumber("Angler/I", controller.getI());
    this.armControlD = new LoggedTunableNumber("Angler/D", controller.getD());
  }

  public void setGoal(Rotation2d goal) {
    controller.setGoal(goal.getDegrees());
  }

  public void reset(TrapezoidProfile.State state) {
    controller.reset(state);
  }

  public void executePIDClamped() {
    double PID = controller.calculate(measureSupplier.get().getDegrees());

    double FF =
        armFF.calculate(controller.getSetpoint().position, controller.getSetpoint().velocity);

    voltageConsumer.accept(
        MathUtil.clamp(PID + FF, AnglerConstants.kMinVoltage, AnglerConstants.kMaxVoltage));
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

  public void updateTunablePID() {
    if (armControlP.hasChanged(hashCode())
        || armControlI.hasChanged(hashCode())
        || armControlD.hasChanged(hashCode())) {
      controller.setP(armControlP.get());
      controller.setI(armControlI.get());
      controller.setD(armControlD.get());
    }
  }
}
