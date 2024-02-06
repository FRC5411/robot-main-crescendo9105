package frc.robot.util;

import frc.robot.subsystems.shooter.Angler.AnglerConstants;

public class TrajectoryAngleSolver {
  public static double xGoalMeters = 0.23;
  public static double yGoalMeters = 2.045;
  public static double heightFromShootMeters = 0.0;
  public static double mdistanceFromWallMeters = 0.0;

  public static double thetaDegrees = 0;

  public static double distanceConstant = 0;
  public static double velocityConstant = 0;

  public static double newtonRaphsonSolver(double distanceFromWallMeters, double initVelocityMPS) {
    mdistanceFromWallMeters = distanceFromWallMeters;
    thetaDegrees = 15;
    int i = 0;
    while (Math.abs(trajectoryByAngle(mdistanceFromWallMeters, initVelocityMPS, thetaDegrees))
            > 1e-2
        && i < 10) {
      thetaDegrees =
          createTangentLineXIntercept(
              thetaDegrees,
              trajectoryByAngle(
                  mdistanceFromWallMeters, initVelocityMPS, Math.toRadians(thetaDegrees)),
              discreteDerivativeOfTrajectoryByAngle(
                  mdistanceFromWallMeters, initVelocityMPS, Math.toRadians(thetaDegrees), 1e-4));
      i++;
    }
    return thetaDegrees;
  }

  public static double trajectoryByAngle(
      double distanceFromWallMeters, double initVelocityMPS, double thetaRadians) {
    setDistanceCAndVelocityC(
        distanceFromWallMeters - AnglerConstants.kPivotLengthMeters * Math.cos(thetaRadians),
        initVelocityMPS);

    return -yGoalMeters
        + distanceConstant * Math.tan(thetaRadians)
        - distanceConstant * distanceConstant * velocityConstant * secantSquared(thetaRadians)
        - AnglerConstants.kPivotLengthMeters * Math.sin(thetaRadians);
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
    setDistanceCAndVelocityC(distanceFromWallMeters, initVelocityMPS);

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
      double distanceFromWallMeters, double initVelocityMPS) {
    distanceConstant = -(xGoalMeters - distanceFromWallMeters);
    velocityConstant = 9.81 / (2 * initVelocityMPS * initVelocityMPS);
  }
}
