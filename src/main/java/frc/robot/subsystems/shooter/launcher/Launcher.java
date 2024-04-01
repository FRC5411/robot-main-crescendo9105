package frc.robot.subsystems.shooter.launcher;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.managers.RobotSetpoints.LauncherSetpoints;
import frc.robot.utils.debugging.SysIDCharacterization;
import frc.robot.utils.math.LinearProfile;

public class Launcher extends SubsystemBase {
    private LauncherIO launcherIO;
    private LauncherIOInputsAutoLogged launcherIOInputs = new LauncherIOInputsAutoLogged();

    private LinearProfile topWheelProfile = new LinearProfile(50, 0.02);
    private LinearProfile bottomWheelProfile = new LinearProfile(50, 0.02);

    private LauncherSetpoints launcherSetpointMPS = null;

    public Launcher(LauncherIO launcherIO) {
        this.launcherIO = launcherIO;
    }

    @Override
    public void periodic() {
        launcherIO.updateInputs(launcherIOInputs);
        Logger.processInputs("Shooter/Launcher/Inputs", launcherIOInputs);

        if (launcherSetpointMPS != null) {
            // System.out.println("HAHA");
            launcherIO.setTopVelocity(
                topWheelProfile.calculateSetpoint(), topWheelProfile.getCurrentAcceleration());
            launcherIO.setBottomVelocity(
                bottomWheelProfile.calculateSetpoint(), bottomWheelProfile.getCurrentAcceleration());
        }
    }

    public Command setVelocityMPS(LauncherSetpoints velocityMPS) {
        return new InstantCommand(() -> setLauncherVelocityMPS(velocityMPS), this);
    }

    public void setLauncherVelocityMPS(LauncherSetpoints velocityMPS) {
        launcherSetpointMPS = velocityMPS;
        if(launcherSetpointMPS != null) {
          topWheelProfile.setGoal(launcherSetpointMPS.getTopSpeedMPS().getAsDouble(), launcherIOInputs.topFlywheelVelocityMPS);
          bottomWheelProfile.setGoal(launcherSetpointMPS.getBottomSpeedMPS().getAsDouble(), launcherIOInputs.bottomFlywheelVelocityMPS);
        }
    }

    public void setLauncherVolts(double topFlywheelVolts, double bottomFlywheelVolts) {
        launcherIO.setTopVolts(topFlywheelVolts);
        launcherIO.setBottomVolts(bottomFlywheelVolts);
    }

    public void setTopLauncherVolts(double topFlywheelVolts) {
        launcherIO.setTopVolts(topFlywheelVolts);
    }

    public void stopMotors() {
        launcherSetpointMPS = null;
        launcherIO.setTopVolts(0.0);
        launcherIO.setBottomVolts(0.0);
    }

    public boolean atFlywheelSetpoints() {
        return launcherIOInputs.topFlywheelErrorMPS < 1
        && launcherIOInputs.bottomFlywheelErrorMPS < 1;
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
