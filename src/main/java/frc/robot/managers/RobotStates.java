package frc.robot.managers;

public class RobotStates {
  public static enum IntakeStates {
    OFF,
    INTAKE,
    OUTTAKE
  }

  public static enum IndexerStates {
    OFF,
    INDEX,
    OUTDEX,
    STOW,
    AMP
  }

  public static enum ClimbStates {
    OFF,
    IDLE,
    GRAB,
    PULL,
    MOVE_BOTH_UP,
    MOVE_BOTH_DOWN,
    AMP
  }

  public static enum YoshiStates {
    OFF,
    IDLE,
    GROUND_AMP,
    GROUND_INTAKE
  }

  public enum AnglerStates{
    OFF,
    AIM,
    INTAKE,
    CLIMB,
    EJECT,
    UP,
    DOWN,
    IDLE,
    PODIUM,
    SPEAKER,
    FEEDER,
    AMP
  }

  public enum LauncherStates{
    OFF,
    SPEAKER_SHOT,
    IDLE,
    FEEDER,
    REV_AMP,
    SHOOT_AMP,
    EJECT,
    FULL_SPEED
  }

  public static enum ShooterStates {
    OFF,
    AIM,
    INTAKE,
    CLIMB,
    EJECT,
    UP,
    DOWN,
    FIRE,
    IDLE,
    PODIUM,
    SPEAKER,
    FEEDER,
    REV_AMP,
    SHOOT_AMP,
  }
}
