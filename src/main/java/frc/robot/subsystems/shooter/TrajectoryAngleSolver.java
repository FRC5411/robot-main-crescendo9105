package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.shooter.angler.AnglerConstants;
import org.littletonrobotics.junction.Logger;

public class TrajectoryAngleSolver {
  public static double xGoalMeters =
      (DriverStation.getAlliance().isPresent())
          ? (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue))
              ? 0.23
              : 16.5 - 0.023
          : 0.23;
  public static double yGoalMeters = 2.045;
  public static double heightFromShootMeters = 0.25;
  public static double mdistanceFromWallMeters = 0.0;

  public static double thetaDegrees = 0;

  public static double distanceConstant = 0;
  public static double velocityConstant = 0;

  public static double newtonRaphsonSolver(double distanceFromWallMeters, double initVelocityMPS) {
    mdistanceFromWallMeters = distanceFromWallMeters;
    thetaDegrees = 15;
    int i = 0;
    Timer timer = new Timer();
    timer.start();
    while (Math.abs(trajectoryByAngle(mdistanceFromWallMeters, initVelocityMPS, thetaDegrees))
            > 1e-4
        && i < 1000) {
      thetaDegrees =
          createTangentLineXIntercept(
              thetaDegrees,
              trajectoryByAngle(
                  mdistanceFromWallMeters, initVelocityMPS, Math.toRadians(thetaDegrees)),
              discreteDerivativeOfTrajectoryByAngle(
                  mdistanceFromWallMeters, initVelocityMPS, Math.toRadians(thetaDegrees), 1e-4));
      i++;
    }
    Logger.recordOutput("Shot Calculation Time", timer.get());
    timer.stop();
    return thetaDegrees;
  }

  public static double trajectoryByAngle(
      double distanceFromWallMeters, double initVelocityMPS, double thetaRadians) {
    setDistanceCAndVelocityC(distanceFromWallMeters, initVelocityMPS, thetaRadians);
    Logger.recordOutput("Distance Constant", distanceConstant);
    Logger.recordOutput("Velocity Constant", velocityConstant);

    return +distanceConstant * Math.tan(thetaRadians)
        - Math.pow(distanceConstant, 2) * velocityConstant * secantSquared(thetaRadians)
        + AnglerConstants.kPivotLengthMeters * Math.sin(thetaRadians)
        + heightFromShootMeters
        - yGoalMeters;
  }

  public static double discreteDerivativeOfTrajectoryByAngle(
      double distanceFromWallMeters,
      double initVelocityMPS,
      double thetaRadians,
      double deltaTheta) {

    double thetaRadianst1 = thetaRadians;
    double thetaRadianst2 = thetaRadians + deltaTheta;

    return (trajectoryByAngle(distanceFromWallMeters, initVelocityMPS, thetaRadianst2)
            - trajectoryByAngle(distanceFromWallMeters, initVelocityMPS, thetaRadianst1))
        / deltaTheta;
  }

  public static double derivativeOfTrajectoryByAngle(
      double distanceFromWallMeters, double initVelocityMPS, double thetaRadians) {
    setDistanceCAndVelocityC(distanceFromWallMeters, initVelocityMPS, thetaRadians);

    return (Math.PI / 180)
        * (distanceConstant * secantSquared(thetaRadians)
            - 2
                * Math.pow(distanceConstant, 2)
                * velocityConstant
                * secantSquared(thetaRadians)
                * Math.tan(thetaRadians)
            - AnglerConstants.kPivotLengthMeters * Math.cos(Math.toDegrees(thetaDegrees)));
  }

  public static double createTangentLineXIntercept(double x, double y, double slope) {
    return -(y / slope) + x;
  }

  public static double secantSquared(double thetaRadians) {
    return Math.pow(1 / Math.cos(thetaRadians), 2);
  }

  public static void setDistanceCAndVelocityC(
      double distanceFromWallMeters, double initVelocityMPS, double thetaRadians) {
    distanceConstant =
        -(xGoalMeters
            - distanceFromWallMeters
            + AnglerConstants.kPivotLengthMeters * Math.cos(thetaRadians));
    velocityConstant = 9.81 / (2 * initVelocityMPS * initVelocityMPS);
  }
}
