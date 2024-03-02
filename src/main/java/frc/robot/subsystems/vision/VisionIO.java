package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog 
    public class VisionIoInputs{ 
        public static double pitch = 0; 
        public static double area = 0; 
        public static double yaw = 0; 
        public static boolean hasTargets = false;  
        public static Pose2d cameraToTarget = null; 
        public static double latencySeconds = 0; 
    } 

    public default void updateInputs(VisionIoInputs inputs){}
}
