package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CamLLConsts;

public class CameraLL extends SubsystemBase {
    private VisionIOLimelight io;  
    private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();    

    public CameraLL(){ 
        this.io = new VisionIOLimelight(CamLLConsts.key,CamLLConsts.offset, CamLLConsts.debounceTime);
    }  

    public double getTargetYaw(){ 
        return io.getTargetYaw();
    }  

    public double getTargetPitch(){ 
        return io.getTargetPitch();
    }  

    public double getTargetArea(){ 
        return io.getTargetArea();
    }   

    public Pose2d getTargetPose(){ 
        return io.getTargetPose();
    } 

    @Override 
    public void periodic(){ 
        io.updateInputs(inputs); 
        Logger.processInputs(CamLLConsts.key,inputs);
    }

    

  
}
