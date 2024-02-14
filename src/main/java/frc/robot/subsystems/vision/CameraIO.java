package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public interface CameraIO {

  public static class CameraIOLoggedInputs {
    public String cameraName = new String("");
    public int pipeline = 0;
    public double timeStamp = 0.0;

    public boolean hasTarget = false;
    public String targetType = new String("");
    public double targetYaw = 0.0;
    public double targetPitch = 0.0;
    public double targetRoll = 0.0;
    public double targetArea = 0.0;

    public Pose2d targetPose = new Pose2d();
    public int amountOfTargets = 0;
    public double xStdDevs = 0.0;
    public double yStdDevs = 0.0;
    public double thetaStdDevs = 0.0;
  }

  public default void updateInputs(CameraIOLoggedInputs inputs) {}

  public default void setPipeline(int pipeline) {}
}
