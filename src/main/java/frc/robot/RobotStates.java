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
    STOW
  }

  public static enum YoshiStates {
    OFF,
    IDLE,
    GROUND,
    AMP
  }

  public static enum ClimbStates {
    OFF,
    IDLE,
    GRAB,
    PULL
  }

  public static enum ShooterStates {
    OFF,
    AIM,
    INTAKE,
    CLIMB,
    EJECT
  }
}
