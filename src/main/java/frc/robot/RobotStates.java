package frc.robot;

public class RobotStates {
  public static enum IntakeStates {
    OFF,
    INTAKE,
    OUTTAKE,
    AMP
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
    GROUND_INTAKE,
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
    REV_AMP,
    SHOOT_AMP,
    AIM_AUTON,
    INTAKE_AUTON
  }
}
