package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
  public static class LL {
    public static final String centerLLNT = "limelight-limeone";
    public static final String leftLLNT = "limelight-left";
    public static final String rightLLNT = "limelight-right";

    public static final Pose3d rightLLOffsetMeters =
        new Pose3d(
            new Translation3d(0.13 - 0.3 - 0.02 + 0.1, 0.09 - 0.04 + 0.2, 0.0),
            new Rotation3d(Math.toRadians(43 - 5), 0, 0));
    public static final Pose3d leftLLOffsetMeters =
        new Pose3d(
            new Translation3d(-0.13 + 0.3 + 0.02, 0.09 + 0.04 - 0.2, 0.0),
            new Rotation3d(Math.toRadians(43 + 5), 0, 0));

    public static final int apriltagPipelineIndex = 0;
    public static final int gamePiecePipelineIndex = 1;
  }
}
