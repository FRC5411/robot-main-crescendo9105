package frc.robot.subsystems.shooter.LeadScrewArm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ScrewArmConstants {
  public static int kMotorID = 1;
  public static int kEncoderID = 1;

  public static double kPivotLength = 3;
  public static double kDriverLength = 5;

  public static double kS = 0.0;
  public static double kG = 0.0;

  public static double kMaxVoltage = 12.0;

  public static Rotation2d kMinAngle = Rotation2d.fromDegrees(15);
  public static Rotation2d kMaxAngle = Rotation2d.fromDegrees(70);

  public static double kP = 0.0;
  public static double kI = 0.0;
  public static double kD = 0.0;

  public static TrapezoidProfile.Constraints kConstraints = 
    new TrapezoidProfile.Constraints(0, 0);

  public static double kSimP = 0.0;
  public static double kSimI = 0.0;
  public static double kSimD = 0.0;

  public static TrapezoidProfile.Constraints kSimConstraints = 
    new TrapezoidProfile.Constraints(0, 0);
}
