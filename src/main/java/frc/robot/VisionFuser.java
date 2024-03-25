package frc.robot;

import edu.wpi.first.math.VecBuilder;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;
import frc.robot.utils.debugging.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/* Class fuses poses from vision and drive, makes interaction between the poses simpler */
/* Don't delete this, or else working with poses will be a pain in the butt */
public class VisionFuser {
  private Drive robotDrive;
  private Vision robotVision;

  private LoggedTunableNumber visionSingleStdDevX;
  private LoggedTunableNumber visionSingleStdDevY;
  private LoggedTunableNumber visionSingleStdDevTheta;

  private LoggedTunableNumber visionMultiStdDevX;
  private LoggedTunableNumber visionMultiStdDevY;
  private LoggedTunableNumber visionMultiStdDevTheta;

  public VisionFuser(Drive drive, Vision vision) {
    robotDrive = drive;
    robotVision = vision;

    visionSingleStdDevX =
        new LoggedTunableNumber(
            "VisionFuser/SingleStdDevX", vision.getSingleStdDevsCoeff().get(0, 0));
    visionSingleStdDevY =
        new LoggedTunableNumber(
            "VisionFuser/SingleStdDevY", vision.getSingleStdDevsCoeff().get(1, 0));
    visionSingleStdDevTheta =
        new LoggedTunableNumber(
            "VisionFuser/SingleStdDevTheta", vision.getSingleStdDevsCoeff().get(2, 0));

    visionMultiStdDevX =
        new LoggedTunableNumber(
            "VisionFuser/MultiStdDevX", vision.getMultiStdDevsCoeff().get(0, 0));
    visionMultiStdDevY =
        new LoggedTunableNumber(
            "VisionFuser/MultiStdDevY", vision.getMultiStdDevsCoeff().get(1, 0));
    visionMultiStdDevTheta =
        new LoggedTunableNumber(
            "VisionFuser/MultiStdDevTheta", vision.getMultiStdDevsCoeff().get(2, 0));
  }

  public void periodic() {
    if (visionMultiStdDevX.hasChanged(hashCode())
        || visionMultiStdDevY.hasChanged(hashCode())
        || visionMultiStdDevTheta.hasChanged(hashCode())) {
      robotVision.setMultiStdDevs(
          visionMultiStdDevX.get(), visionMultiStdDevY.get(), visionMultiStdDevTheta.get());
    }
    if (visionSingleStdDevX.hasChanged(hashCode())
        || visionSingleStdDevY.hasChanged(hashCode())
        || visionSingleStdDevTheta.hasChanged(hashCode())) {
      robotVision.setSingleStdDevs(
          visionSingleStdDevX.get(), visionSingleStdDevY.get(), visionSingleStdDevTheta.get());
    }

    final VisionIOInputsAutoLogged inputsFrontLeft = robotVision.getInputsFrontLeft();
    final VisionIOInputsAutoLogged inputsFrontRight = robotVision.getInputsFrontRight();
    final VisionIOInputsAutoLogged inputsSideLeft = robotVision.getInputsSideLeft();
    final VisionIOInputsAutoLogged inputsSideRight = robotVision.getInputsSideRight();

    if (inputsFrontLeft.hasTarget) {
      robotDrive.addVisionMeasurement(
          inputsFrontLeft.estimatedRobotPose,
          inputsFrontLeft.latestTimestamp,
          VecBuilder.fill(
              inputsFrontLeft.xStandardDeviation,
              inputsFrontLeft.yStandardDeviation,
              inputsFrontLeft.thetaStandardDeviation));
    }

    if (inputsFrontRight.hasTarget) {
      robotDrive.addVisionMeasurement(
          inputsFrontRight.estimatedRobotPose,
          inputsFrontRight.latestTimestamp,
          VecBuilder.fill(
              inputsFrontRight.xStandardDeviation,
              inputsFrontRight.yStandardDeviation,
              inputsFrontRight.thetaStandardDeviation));
    }

    if (inputsSideLeft.hasTarget) {
      robotDrive.addVisionMeasurement(
          inputsSideLeft.estimatedRobotPose,
          inputsSideLeft.latestTimestamp,
          VecBuilder.fill(
              inputsSideLeft.xStandardDeviation,
              inputsSideLeft.yStandardDeviation,
              inputsSideLeft.thetaStandardDeviation));
    }

    if (inputsSideRight.hasTarget) {
      robotDrive.addVisionMeasurement(
          inputsSideRight.estimatedRobotPose,
          inputsSideRight.latestTimestamp,
          VecBuilder.fill(
              inputsSideRight.xStandardDeviation,
              inputsSideRight.yStandardDeviation,
              inputsSideRight.thetaStandardDeviation));
    }

    Logger.recordOutput(
        "VisionFuse/FrontLeftTransform",
        robotVision.getInputsFrontLeft().estimatedRobotPose.minus(robotDrive.getOdometryPose()));
    Logger.recordOutput(
        "VisionFuse/FrontRightTransform",
        robotVision.getInputsFrontRight().estimatedRobotPose.minus(robotDrive.getOdometryPose()));
    Logger.recordOutput(
        "VisionFuse/SideLeftTransform",
        robotVision.getInputsSideLeft().estimatedRobotPose.minus(robotDrive.getOdometryPose()));
    Logger.recordOutput(
        "VisionFuse/SideRightTransform",
        robotVision.getInputsSideRight().estimatedRobotPose.minus(robotDrive.getOdometryPose()));
  }
}
