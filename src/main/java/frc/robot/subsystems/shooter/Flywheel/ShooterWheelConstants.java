package frc.robot.subsystems.shooter.Flywheel;

public class ShooterWheelConstants {
  public static int kTopMotorID = 31;
  public static int kBottomMotorID = 32;

  public static double kRadiusM = 7.62 / 100;
  public static double kCircumferenceM = 2 * Math.PI * kRadiusM;
  public static double kGearing = 1.0 / 1.0;
  public static double kJKgMetersSquared = 0.0002;

  public static double kP = 0.0;
  public static double kI = 0.0;
  public static double kD = 0.0;

  public static double kS = 0.0;
  public static double kV = 0.237;
  public static double kA = 0.0;
  public static double kFlywheelRateLimit = 25;

  public static double kSimP = 0.19;
  public static double kSimI = 0.0;
  public static double kSimD = 0.0;

  public static double kSimS = 0.0;
  public static double kSimV = 0.237;
  public static double kSimA = 0.0;

  public static double kPassiveMPS = 15.0;
  public static double kShootMPS = 30.0;
}
