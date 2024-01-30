package frc.robot.subsystems.shooter.Flywheel;

public class ShooterWheelConstants {
  public static int kTopMotorID = 21;
  public static int kBottomMotorID = 22;

  public static double kRadiusM = 5.08 / 100;
  public static double kCircumferenceM = 2 * Math.PI * kRadiusM;
  public static double kGearing = 1.0 / 1.0;
  public static double kJKgMetersSquared = 0.0001;

  public static double kP = 0.0;
  public static double kI = 0.0;
  public static double kD = 0.0;

  public static double kS = 0.0;
  public static double kV = 0.0;
  public static double kA = 0.0;
  public static double kFlywheelRateLimit = 2.5;

  public static double kSimP = 0.2;
  public static double kSimI = 0.0;
  public static double kSimD = 0.0;

  public static double kSimS = 0.0;
  public static double kSimV = 2.0;
  public static double kSimA = 0.0;
}
