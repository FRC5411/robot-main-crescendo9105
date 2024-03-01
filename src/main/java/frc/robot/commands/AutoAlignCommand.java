package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.debugging.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class AutoAlignCommand {
  private static SlewRateLimiter driveXLimiter = new SlewRateLimiter(5);
  private static SlewRateLimiter driveYLimiter = new SlewRateLimiter(5);
  private static ProfiledPIDController driveThetaController =
      new ProfiledPIDController(5, 0, 0, new Constraints(300, 200));

  private static LoggedTunableNumber driveThetaP =
      new LoggedTunableNumber("AutoAlign/DriveTheta/P", driveThetaController.getP());
  private static LoggedTunableNumber driveThetaI =
      new LoggedTunableNumber("AutoAlign/DriveTheta/I", driveThetaController.getI());
  private static LoggedTunableNumber driveThetaD =
      new LoggedTunableNumber("AutoAlign/DriveTheta/D", driveThetaController.getD());
  private static LoggedTunableNumber driveThetaV =
      new LoggedTunableNumber(
          "AutoAlign/DriveTheta/V", driveThetaController.getConstraints().maxVelocity);
  private static LoggedTunableNumber driveThetaA =
      new LoggedTunableNumber(
          "AutoAlign/DriveTheta/A", driveThetaController.getConstraints().maxAcceleration);

  public static Command angleToSpeakerCommand(Drive robotDrive) {
    driveThetaController.setTolerance(0.2);
    driveThetaController.enableContinuousInput(0, 360);

    return new FunctionalCommand(
        () -> {
          final ChassisSpeeds desiredChassisSpeeds = robotDrive.getDesiredChassisSpeeds();
          driveXLimiter.reset(desiredChassisSpeeds.vxMetersPerSecond);
          driveYLimiter.reset(desiredChassisSpeeds.vyMetersPerSecond);

          // Plus and negative logic have to be tested based of gyro readings
          driveThetaController.reset(robotDrive.getRotation().getDegrees());
        },
        () -> {
          double desiredXSpeed = driveXLimiter.calculate(0);
          double desiredYSpeed = driveYLimiter.calculate(0);

          double desiredThetaDegrees =
              driveThetaController.calculate(
                  robotDrive.getPosition().getRotation().getDegrees(),
                  Math.toDegrees(
                          Math.atan2(
                              Constants.kSpeaker3DPose.getY() - robotDrive.getPosition().getY(),
                              Constants.kSpeaker3DPose.getX() - robotDrive.getPosition().getX()))
                      + 180);

          Logger.recordOutput("AutoAlign/DesiredX", desiredXSpeed);
          Logger.recordOutput("AutoAlign/DesiredY", desiredYSpeed);
          Logger.recordOutput("AutoAlign/DesiredTheta", desiredThetaDegrees);
          Logger.recordOutput("AutooAlign/Controller/Goal", driveThetaController.getGoal().position);
          Logger.recordOutput(
              "AutoAlign/Controller/Setpoint", driveThetaController.getSetpoint().position);
          Logger.recordOutput(
              "AutoAlign/Controller/Measure", robotDrive.getPosition().getRotation().getDegrees());
          robotDrive.runSwerve(
              new ChassisSpeeds(desiredXSpeed, desiredYSpeed, Math.toRadians(desiredThetaDegrees)));
        },
        (interrupted) -> {},
        () -> false,
        robotDrive);
  }

  public static void updateTunables() {
    if (driveThetaP.hasChanged(driveThetaP.hashCode())
        || driveThetaI.hasChanged(driveThetaI.hashCode())
        || driveThetaD.hasChanged(driveThetaD.hashCode())) {
      driveThetaController.setP(driveThetaP.get());
      driveThetaController.setI(driveThetaI.get());
      driveThetaController.setD(driveThetaD.get());
    }
    if (driveThetaV.hasChanged(driveThetaV.hashCode())
        || driveThetaA.hasChanged(driveThetaA.hashCode())) {
      driveThetaController.setConstraints(new Constraints(driveThetaV.get(), driveThetaA.get()));
    }
  }

  public static void initLogTables() {
    Logger.recordOutput("AutAlign/DesiredX", 0);
    Logger.recordOutput("AutAlign/DesiredY", 0);
    Logger.recordOutput("AutAlign/DesiredTheta", 0);
    Logger.recordOutput("AutoAlign/Controller/Goal", 0);
    Logger.recordOutput("AutoAlign/Controller/Setpoint", 0);
    Logger.recordOutput("AutoAlign/Controller/Measure", 0);
  }
}
