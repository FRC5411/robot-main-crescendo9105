package frc.robot.subsystems.shooter.Flywheel;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterWheelSimIO implements ShooterWheelIO {
    private FlywheelSim flywheelMotorLeft = 
        new FlywheelSim(
            DCMotor.getNEO(1), 
            1,
            0,
            VecBuilder.fill( 0.1 ) );
    private FlywheelSim flywheelMotorRight = 
        new FlywheelSim(
            DCMotor.getNEO(1),
            1,
            0, 
            VecBuilder.fill( 0.1 ) );

    public double flywheelAppliedVoltsLeft = 0;
    public double flywheelVelocityMeasuredMPSLeft = 0;
    public double flywheelVelocitySetpointMPSLeft = 0;

    public double flywheelAppliedVoltsRight = 0;
    public double flywheelVelocityMeasuredMPSRight = 0;
    public double flywheelVelocitySetpointMPSRight = 0;

    PIDController flywheelPIDLeft = new PIDController(0.1, 0, 0);
    SimpleMotorFeedforward flywheelFeedforwardLeft = new SimpleMotorFeedforward(0.0, 0.1);

    PIDController flywheelPIDRight = new PIDController(0.1, 0, 0);
    SimpleMotorFeedforward flywheelFeedforwardRight = new SimpleMotorFeedforward(0.0, 0.1);

    @Override
    public void updateInputs(ShooterWheelIOInputs inputs) {
        flywheelVelocityMeasuredMPSLeft = 
            (flywheelMotorLeft.getAngularVelocityRPM() * ShooterWheelConstants.circumferenceM) / 60;
        inputs.flywheelVelocityMPSLeft = flywheelVelocityMeasuredMPSLeft;
        inputs.flywheelAppliedVoltsLeft = flywheelAppliedVoltsLeft;
        inputs.flywheelCurrentAmpsLeft = new double[] { flywheelMotorLeft.getCurrentDrawAmps() };
        inputs.flywheelVelocityMPSLeftSetpoint = flywheelVelocitySetpointMPSLeft;
        
        flywheelVelocityMeasuredMPSRight = 
            (flywheelMotorRight.getAngularVelocityRPM() * ShooterWheelConstants.circumferenceM) / 60;
        inputs.flywheelVelocityMPSRight = flywheelVelocityMeasuredMPSRight;
        inputs.flywheelAppliedVoltsRight = flywheelAppliedVoltsRight;
        inputs.flywheelCurrentAmpsRight = new double[] { flywheelMotorRight.getCurrentDrawAmps() };
        inputs.flywheelVelocityMPSRightSetpoint = flywheelVelocitySetpointMPSRight;
        
        flywheelMotorLeft.update(0.02);
        flywheelMotorRight.update(0.02);
    }

    @Override
    public void setFlywheelsVoltsLeft(double volts) {
        flywheelAppliedVoltsLeft = volts;
        flywheelMotorLeft.setInputVoltage(volts);
    }

    @Override
    public void setFlywheelsVoltsRight(double volts) {
        flywheelAppliedVoltsRight = volts;
        flywheelMotorRight.setInputVoltage(volts);
    }

    @Override
    public void setFlywheelsVelocityLeft(double velocityMPS) {
        flywheelVelocitySetpointMPSLeft = velocityMPS;
        setFlywheelsVoltsLeft(
            flywheelFeedforwardLeft.calculate( velocityMPS) + 
            flywheelPIDLeft.calculate( flywheelVelocityMeasuredMPSLeft, velocityMPS ) );
    }

    @Override
    public void setFlywheelsVelocityRight(double velocityMPS) {
        flywheelVelocitySetpointMPSRight = velocityMPS;
        setFlywheelsVoltsRight(
            flywheelFeedforwardRight.calculate( velocityMPS) + 
            flywheelPIDRight.calculate( flywheelVelocityMeasuredMPSRight, velocityMPS ) );
    }
}