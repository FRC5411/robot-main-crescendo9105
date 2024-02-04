package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

public class ShooterConstants {
  public static final Pose3d kSpeaker3DPose =
      new Pose3d(
          new Translation3d(
              (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                  ? 0.23
                  : 16.5 - 0.23,
              8.27 / 2 + 1.4478,
              2.045),
          new Rotation3d());
}
