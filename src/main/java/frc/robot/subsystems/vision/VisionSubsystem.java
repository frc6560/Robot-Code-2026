package frc.robot.subsystems.vision;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: add disabling cameras individually
// add disabling vision
public class VisionSubsystem extends SubsystemBase{
    private final List<LimelightVision> visionList;

    /** Wrapper class for all limelights */
    public VisionSubsystem(List<LimelightVision> visionList){
        this.visionList = visionList;
    }

    @Override
    public void periodic(){
        for(LimelightVision vision : visionList){
            vision.update();
        }
    }

    public void hardReset(String cameraName){
        for(LimelightVision vision : visionList){
            if(vision.getName().equals(cameraName)){
                vision.hardUpdate();
            }
        }
    }

    public void disableVision(){
        for(LimelightVision vision : visionList){
            vision.disableVision(true);
        }
    }

     public void enableVision(){
        for(LimelightVision vision : visionList){
            vision.disableVision(false);
        }
    }
}
