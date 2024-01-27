package frc.robot.util;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.shooter.LeadScrewArm.ScrewArmConstants;

public class ScrewArmController {
    public final Supplier<Rotation2d> measureSupplier;
    public final Consumer<Double> voltageConsumer;
    public final ProfiledPIDController controller;

    public ScrewArmController(
        Supplier<Rotation2d> measureSupplier, Consumer<Double> voltageConsumer, 
        ProfiledPIDController controller) {
        this.measureSupplier = measureSupplier;
        this.voltageConsumer = voltageConsumer;
        this.controller = controller;
    }

    public void setGoal(Rotation2d goal) {
        controller.setGoal( goal.getDegrees() );
    }

    public void reset(TrapezoidProfile.State state) {
        controller.setGoal(state);
    }

    public void initPID(Rotation2d measure) {
        controller.reset( measureSupplier.get().getDegrees() );
    }

    public void executePID(Rotation2d goal) {
        double PID = 
            controller.calculate( measureSupplier.get().getDegrees() ) 
            * ScrewArmKinematics.scaleVoltage(
                Rotation2d.fromDegrees(
                    controller.getSetpoint().position ) );

        double FF = 
            Math.signum(controller.getSetpoint().velocity) * ScrewArmConstants.kScrewArmKS
            + ScrewArmConstants.kScrewArmKG * ScrewArmKinematics.getGravityVector(
                Rotation2d.fromDegrees( controller.getSetpoint().position ) );

        voltageConsumer.accept(PID + FF);
    }    
}
