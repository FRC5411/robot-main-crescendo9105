package frc.robot;

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
    AMP
  }
}
