package frc.robot.subsystems.shooter.launcher;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.shooter.Shooter.LauncherSetpoints;
import frc.robot.subsystems.shooter.launcher.LauncherIO.LauncherIOInputs;
import frc.robot.utils.debugging.SysIDCharacterization;
import frc.robot.utils.math.LinearProfile;

public class Launcher {
    private LauncherIO launcherIO;
    private LauncherIOInputsAutoLogged launcherIOInputs = new LauncherIOInputsAutoLogged();

    private LinearProfile topWheelProfile = new LinearProfile(50, 0.02);
    private LinearProfile bottomWheelProfile = new LinearProfile(50, 0.02); 
    
      private LauncherSetpoints launcherSetpointMPS = null;
    
    public Launcher(LauncherIO launcherIO) {
        this.launcherIO = launcherIO;
    }

    public void periodic() {
        launcherIO.updateInputs(launcherIOInputs);
        Logger.processInputs("Shooter/Launcher/Inputs", launcherIOInputs);

        if (launcherSetpointMPS != null) {
            launcherIO.setTopVelocity(
                topWheelProfile.calculateSetpoint(), topWheelProfile.getCurrentAcceleration());
            launcherIO.setBottomVelocity(
                bottomWheelProfile.calculateSetpoint(), bottomWheelProfile.getCurrentAcceleration());
        }
    }

    public void setLauncherVelocityMPS(LauncherSetpoints launcherSetpointsMPS) {
        this.launcherSetpointMPS = launcherSetpointsMPS;
        topWheelProfile.setGoal(
            this.launcherSetpointMPS.getTopSpeedMPS().getAsDouble(), 
            launcherIOInputs.topFlywheelVelocityMPS);
        bottomWheelProfile.setGoal(
            this.launcherSetpointMPS.getBottomSpeedMPS().getAsDouble(), 
            launcherIOInputs.bottomFlywheelVelocityMPS);
    }

    public void setLauncherVolts(double topFlywheelVolts, double bottomFlywheelVolts) {
        launcherIO.setTopVolts(topFlywheelVolts);
        launcherIO.setBottomVolts(bottomFlywheelVolts);
    }

    public void setLauncherVoltsManually(double topFlywheelVolts, double bottomFlywheelVolts) {
        launcherSetpointMPS = null;
        launcherIO.setTopVolts(topFlywheelVolts);
        launcherIO.setBottomVolts(bottomFlywheelVolts);
    }

    public void stopMotors() {
        setLauncherVolts(0.0, 0.0);
    }

    public void stopMotorsManually() {
        setLauncherVoltsManually(0.0, 0.0);
    }

    public double getTopLauncherError() {
        return launcherIOInputs.topFlywheelErrorMPS;
    }

    public double getBottomLauncherError() {
        return launcherIOInputs.bottomFlywheelErrorMPS;
    }

    public boolean isAtSetpoint() {
        return 
            getTopLauncherError() < 0.5 && 
            getBottomLauncherError() < 0.5;
    }

    public LauncherIOInputs getLauncerInputs() {
        return launcherIOInputs;
    }

    public Command characterizeFlywheel(Subsystem subsystem) {
        return SysIDCharacterization.runShooterSysIDTests(
            (volts) -> {
                launcherSetpointMPS = null;
                launcherIO.setTopVolts(volts);
                launcherIO.setBottomVolts(volts);
        }, subsystem);
    }
}
