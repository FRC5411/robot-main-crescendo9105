package frc.robot.util;

public class TrajectoryAngleSolver {
  public static double xGoalMeters = 0.23;
  public static double yGoalMeters = 2.045;
  public static double heightFromShootMeters = 0.25;
  public static double c = 0;
  public static double v = 0;

  public static double newtonRaphsonSolver(double distanceFromWallMeters, double initVelocityMPS) {
    double thetaDegrees = 15;
    int i = 0;
    while (Math.abs(trajectoryByAngle(distanceFromWallMeters, initVelocityMPS, thetaDegrees)) > 1e-2
        && i < 10) {
      thetaDegrees =
          createTangentLineXIntercept(
              thetaDegrees,
              trajectoryByAngle(
                  distanceFromWallMeters, initVelocityMPS, Math.toRadians(thetaDegrees)),
              derivativeOfTrajectoryByAngle(
                  distanceFromWallMeters, initVelocityMPS, Math.toRadians(thetaDegrees)));
      i++;
    }
    return thetaDegrees;
  }

  public static double trajectoryByAngle(
      double distanceFromWallMeters, double initVelocityMPS, double thetaRadians) {
    setCAndV(distanceFromWallMeters, initVelocityMPS);

    return heightFromShootMeters
        - yGoalMeters
        + c * Math.tan(thetaRadians)
        - c * c * v * secantSquared(thetaRadians);
  }

  public static double derivativeOfTrajectoryByAngle(
      double distanceFromWallMeters, double initVelocityMPS, double thetaRadians) {
    setCAndV(distanceFromWallMeters, initVelocityMPS);

    return (Math.PI / 180)
        * (c * secantSquared(thetaRadians)
            - 2 * c * c * v * secantSquared(thetaRadians) * Math.tan(thetaRadians));
  }

  public static double createTangentLineXIntercept(double x, double y, double slope) {
    return -(y / slope) + x;
  }

  public static double secantSquared(double thetaRadians) {
    return Math.pow(1 / Math.cos(thetaRadians), 2);
  }

  public static void setCAndV(double distanceFromWallMeters, double initVelocityMPS) {
    c = -(xGoalMeters - distanceFromWallMeters);
    v = 9.81 / (2 * initVelocityMPS * initVelocityMPS);
  }
}
