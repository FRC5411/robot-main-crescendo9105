package frc.robot.subsystems.shooter.Angler;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AnglerConstants {
  public static int kMotorID = 41;
  public static int kAbsoluteEncoderID = 7;
  public static int kRelativeEncoderAID = 8;
  public static int kRelativeEncoderBID = 9;

  public static double kPivotLengthMeters = 0.4826;
  public static double kJKGMetersSquared = 0.167248163371;

  public static double kMaxVoltage = 12.0;
  public static double kMinVoltage = -12.0;

  public static Rotation2d kMinAngle = Rotation2d.fromDegrees(15);
  public static Rotation2d kMaxAngle = Rotation2d.fromDegrees(70);

  public static double kP = 0.0;
  public static double kI = 0.0;
  public static double kD = 0.0;

  public static double kS = 0.0;
  public static double kG = 0.0;
  public static double kV = 0.0;
  public static double kA = 0.0;

  public static TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(0, 0);

  public static double kSimP = 0.25;
  public static double kSimI = 0.0;
  public static double kSimD = 0.0;

  public static double kSimS = 0.0;
  public static double kSimG = 0.0;
  public static double kSimV = 0.0;
  public static double kSimA = 0.0;

  public static TrapezoidProfile.Constraints kSimConstraints =
      new TrapezoidProfile.Constraints(360, 180);
}
