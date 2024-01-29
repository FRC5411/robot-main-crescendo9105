package frc.robot.subsystems.shooter.Flywheel;

public class ShooterWheelConstants {
  public static int kLeftMotorID = 1;
  public static int kRightMotorID = 2;


  public static double kRadiusM = 5.08 / 100;
  public static double kCircumferenceM = 2 * Math.PI * kRadiusM;
  public static double kGearing = 1.0 / 1.0;
  public static double kJKgMetersSquared = 0.0001;

  public static double kPLeft = 0.0;
  public static double kILeft = 0.0;
  public static double kDLeft = 0.0;

  public static double kSLeft = 0.0;
  public static double kVLeft = 0.0;
  public static double kALeft = 0.0;
  public static double kFlywheelRateLimitLeft = 0.5;
  public static double kFlywheelRateLimitRight = 0.5;
}
