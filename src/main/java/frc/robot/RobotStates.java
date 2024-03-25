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

  // public static enum ShooterStates {
  //   OFF,
  //   AIM,
  //   INTAKE,
  //   CLIMB,
  //   EJECT,
  //   UP,
  //   DOWN,
  //   FIRE,
  //   IDLE,
  //   PODIUM,
  //   SPEAKER,
  //   FEEDER,
  //   REV_AMP,
  //   SHOOT_AMP,
  //   AIM_AUTON
  // }

  public static enum LauncherStates {
    OFF,
    IDLE,
    REV_AMP,
    SCORE_AMP,
    EJECT,
    FEEDER,
    SHOOT,
    FULL_SPEED,
  }

  public static enum AnglerStates {
    OFF,
    IDLE,
    AIM_IDLE,
    AIM,
    PODIUM,
    SPEAKER,
    AMP,
    UP,
    DOWN,
    FEEDER,
    EJECT,
    INTAKE,
    CLIMB
  }
}
