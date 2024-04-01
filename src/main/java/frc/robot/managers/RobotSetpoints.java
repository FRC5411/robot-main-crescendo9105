package frc.robot.managers;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.shooter.angler.Angler;
import frc.robot.utils.debugging.LoggedTunableNumber;

public class RobotSetpoints {
    private static LoggedTunableNumber angleTunableNumber = new LoggedTunableNumber("Shooter/AngleDebuggingDegrees", 0.0);

    public static enum AnglerSetpoints {
        AIM(() -> TargetingSystem.getInstance().getLaunchMapAngle()),
        CLIMB(() -> Rotation2d.fromDegrees(25.0)),
        INTAKE(() -> Rotation2d.fromDegrees(45.0)),
        IDLE(() -> Angler.getLastAnglerPosition()),
        PODIUM(() -> Rotation2d.fromDegrees(34.5)),
        SPEAKER(() -> Rotation2d.fromDegrees(57)),
        AMP(() -> Rotation2d.fromDegrees(54.0)),
        FEEDER(() -> Rotation2d.fromDegrees(45.0)),
        DEBUGGING(() -> Rotation2d.fromDegrees(angleTunableNumber.get()));

        private Supplier<Rotation2d> angleSupplier;

        AnglerSetpoints(Supplier<Rotation2d> angle) {
            this.angleSupplier = angle;
        }

        public Supplier<Rotation2d> getAngle() {
            return angleSupplier;
        }
    }

    public static enum LauncherSetpoints {
        EJECT(() -> 5.0, () -> 5.0),
        IDLE(() -> 9.0, () -> 9.0),
        SPEAKER_SHOT(() -> 38.0, () -> 38.0),
        FULL_SPEED(() -> 42.0, () -> 42.0),
        FEEDER(() -> 33.6, () -> 33.6),
        // 12.5: 7 / 9
        // 12.65: 08
        // 12.58: 9 / 12
        // AMP(() -> -0.1, () -> 12.54),
        // AMP(() -> -4.5, () -> 12.0): 5 / 8
        AMP(() -> -4.5, () -> 12.0),
        OFF(() -> 0.0, () -> 0.0);

        private DoubleSupplier topSpeedSupplierMPS;
        private DoubleSupplier bottomSpeedSupplierMPS;

        LauncherSetpoints(DoubleSupplier topSpeedMPS, DoubleSupplier bottomSpeedMPS) {
            this.topSpeedSupplierMPS = topSpeedMPS;
            this.bottomSpeedSupplierMPS = bottomSpeedMPS;
        }

        public DoubleSupplier getTopSpeedMPS() {
            return topSpeedSupplierMPS;
        }

        public DoubleSupplier getBottomSpeedMPS() {
            return bottomSpeedSupplierMPS;
        }
    }

    public static enum YoshivatorSetpoints {
        IDLE(() -> Rotation2d.fromDegrees(100.0), () -> 0.0),
        GROUND_INTAKE(() -> Rotation2d.fromDegrees(-33.5), () -> -12.0),
        DEBUGGING(() -> Rotation2d.fromDegrees(new LoggedTunableNumber("e", 0.0).get()), () -> 0.0);

        private Supplier<Rotation2d> pivotSetpointRotation;
        private Supplier<Double> rollerSetpointVolts;

        YoshivatorSetpoints(
            Supplier<Rotation2d> pivotSetpointRotation, Supplier<Double> rollerSetpointVolts) {
        this.pivotSetpointRotation = pivotSetpointRotation;
        this.rollerSetpointVolts = rollerSetpointVolts;
        }

        public Supplier<Rotation2d> getPivotRotation() {
            return this.pivotSetpointRotation;
        }

        public Supplier<Double> getRollerVolts() {
            return this.rollerSetpointVolts;
        }
    }
    
    public static enum IntakeSetpoint {
        IN(12.0),
        OUT(-12.0),
        STOP(6.0),
        OFF(0.0);
    
        private double volts;
    
        IntakeSetpoint(double volts) {
          this.volts = volts;
        }
    
        public double getVolts() {
          return this.volts;
        }
    }    

    public static enum IndexerSetpoint {
        IN(12.0),
        OUT(-12.0),
        OFF(0.0),
        STOW(2.0),
        AMP(7.0);
    
        private double volts;
    
        IndexerSetpoint(double volts) {
          this.volts = volts;
        }
    
        public double getVolts() {
          return this.volts;
        }
    }

    public static enum ClimbVoltSetpoints {
        BOTH_UP(12.0, 12.0),
        BOTH_DOWN(-12.0, -12.0),
        OFF(0.0, 0.0);

        private double leftVolts;
        private double rightVolts;

        ClimbVoltSetpoints(double leftVolts, double rightVolts) {
            this.leftVolts = leftVolts;
            this.rightVolts = rightVolts;
        }

        public double getLeftVolts() {
            return leftVolts;
        }

        public double getRightVolts() {
            return rightVolts;
        }
    }

    public static enum ClimPosSetpoints {
        IDLE(Rotation2d.fromDegrees(5.0));

        private Rotation2d m_setpoint;

        ClimPosSetpoints(Rotation2d setpoint) {
            this.m_setpoint = setpoint;
        }

        public Rotation2d getRotation() {
            return m_setpoint;
        }
    }
}
