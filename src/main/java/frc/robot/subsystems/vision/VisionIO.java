package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog 
    public class VisionIoInputs{ 
        public double pitch = 0; 
        public double area = 0; 
        public double yaw = 0; 
        public boolean hasTargets = false;  
        public Pose2d cameraToTarget = null; 
        public double latencySeconds = 0; 
    } 

    public default void updateInputs(VisionIoInputs inputs){}
}
