package frc.lib;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
  private NetworkTable limelight;
  private Pose3d offset;
  private Debouncer debouncer;
  private boolean pipelineIndex;

  public Limelight(String key, Pose3d offset, double time) {

    this.offset = offset;
    debouncer = new Debouncer(time);
    limelight = NetworkTableInstance.getDefault().getTable(key);
    setPipelineIndex(1);
  }

  public void setPipelineIndex(int index) {
    limelight.getEntry("getpipe").setNumber(index);
    limelight.getEntry("pipeline").setNumber(index);
  }

  public int getPipeLineIndex() {
    return pipelineIndex ? 1 : 0;
  }

  public boolean hasTarget() {
    return (limelight.getEntry("tv").getDouble(0) == 1);
  }

  public boolean hasTargetDebounced() {
    return debouncer.calculate(hasTarget());
  }

  public String getObjectType() {
    return limelight.getEntry("tclass").getString("cube");
  }

  public double getYaw() {
    return limelight.getEntry("tx").getDouble(0);
  }

  public double getPitch() {
    return limelight.getEntry("ty").getDouble(0);
  }

  public double getArea() {
    return limelight.getEntry("ta").getDouble(0);
  }

  public Pose2d getPose() {
    double[] posevalues = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

    Translation2d translate =
        new Translation2d(posevalues[0] - offset.getX(), posevalues[1] - offset.getY());
    Rotation2d rotation = Rotation2d.fromDegrees(posevalues[3] - offset.getRotation().getX());

    return new Pose2d(translate, rotation);
  }

  public Pose2d getTarget() {
    double[] posevalues =
        limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
    Translation2d translate =
        new Translation2d(posevalues[0] - offset.getX(), posevalues[1] - offset.getY());
    Rotation2d rotation =
        new Rotation2d(Math.toRadians(posevalues[3]) - offset.getRotation().getX());
    return new Pose2d(translate, rotation);
  }

  /** returns latency in seconds (tl + cl) */
  public double getLatency() {
    return (limelight.getEntry("tl").getDouble(0) + limelight.getEntry("cl").getDouble(0)) / 1000.0;
  }

  public void periodic() {}
}
