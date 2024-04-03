package frc.robot.managers;

public class RobotDesiredStates {
  public static enum IntakeStates {
    OFF,
    INTAKE,
    AUTO_INTAKE,
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
    MOVE_BOTH_UP,
    MOVE_BOTH_DOWN,
  }

  public static enum YoshiStates {
    OFF,
    IDLE,
    INTAKE
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
}
