package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;

public class Shooter {
    public static enum AnglerSetpoints {
        AIM(() -> TargetingSystem.getInstance().getLaunchMapAngle()),
        CLIMB(() -> Rotation2d.fromDegrees(25.0)),
        INTAKE(() -> Rotation2d.fromDegrees(45.0)),
        IDLE(() -> new Rotation2d()),
        PODIUM(() -> Rotation2d.fromDegrees(34.5)),
        SPEAKER(() -> Rotation2d.fromDegrees(57)),
        AMP(() -> Rotation2d.fromDegrees(52.0)),
        FEEDER(() -> Rotation2d.fromDegrees(45));
    
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
        AMP(() -> 0.0, () -> 12.6),
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
}
