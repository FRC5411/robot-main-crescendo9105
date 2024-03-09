package frc.robot;

import edu.wpi.first.math.VecBuilder;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

/* Class fuses poses from vision and drive, makes interaction between the poses simpler */
/* Don't delete this, or else working with poses will be a pain in the but */
public class VisionFuser {
    private Drive robotDrive;
    private Vision robotVision;

    public VisionFuser(Drive drive, Vision vision) {
        robotDrive = drive;
        robotVision = vision;
    }

  public void periodic() {
    robotDrive.addVisionMeasurement(
        robotVision.getInputsLeft().estimatedRobotPose,
        robotVision.getInputsLeft().latestTimestamp,
        VecBuilder.fill(
            robotVision.getInputsLeft().xStandardDeviation,
            robotVision.getInputsLeft().yStandardDeviation,
            robotVision.getInputsLeft().thetaStandardDeviation));

    robotDrive.addVisionMeasurement(
        robotVision.getInputsLeft().estimatedRobotPose,
        robotVision.getInputsLeft().latestTimestamp,
        VecBuilder.fill(
            robotVision.getInputsLeft().xStandardDeviation,
            robotVision.getInputsLeft().yStandardDeviation,
            robotVision.getInputsLeft().thetaStandardDeviation));

    Logger.recordOutput(
        "VisionFuse/LeftTransform",
        robotVision.getInputsLeft().estimatedRobotPose.minus(robotDrive.getOdometryPose()));
    Logger.recordOutput(
        "VisionFuse/RightTransform",
        robotVision.getInputsRight().estimatedRobotPose.minus(robotDrive.getOdometryPose()));
  }
}
