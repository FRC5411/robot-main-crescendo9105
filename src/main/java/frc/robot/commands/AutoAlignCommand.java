package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class AutoAlignCommand {
  private static SlewRateLimiter driveXLimiter = new SlewRateLimiter(5);
  private static SlewRateLimiter driveYLimiter = new SlewRateLimiter(5);
  private static ProfiledPIDController driveThetaController =
      new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(300, 200));

  public static Command angleToSpeakerCommand(Drive robotDrive) {
    driveThetaController.setTolerance(0);
    return new FunctionalCommand(
        () -> {
          final ChassisSpeeds desiredChassisSpeeds = robotDrive.getDesiredChassisSpeeds();
          driveXLimiter.reset(desiredChassisSpeeds.vxMetersPerSecond);
          driveYLimiter.reset(desiredChassisSpeeds.vyMetersPerSecond);

          // Plus and negative logic have to be tested based of gyro readings
          driveThetaController.reset(180 + robotDrive.getRotation().getDegrees());
        },
        () -> {
          double desiredXSpeed = driveXLimiter.calculate(0);
          double desiredYSpeed = driveYLimiter.calculate(0);

          double desiredThetaDegrees =
              driveThetaController.calculate(
                  robotDrive.getRotation().getDegrees(),
                  Math.toDegrees(
                      Math.atan2(
                          Constants.kSpeaker3DPose.getY() - robotDrive.getPosition().getY(),
                          Constants.kSpeaker3DPose.getX() - robotDrive.getPosition().getX())));

          robotDrive.runSwerve(
              new ChassisSpeeds(desiredXSpeed, desiredYSpeed, Math.toRadians(desiredThetaDegrees)));
        },
        (interrupted) -> {},
        () -> false,
        robotDrive);
  }
}
