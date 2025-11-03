package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.List;

public class LLsubsystem {
    Limelight3A limelight;
    public LLsubsystem(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, RobotConstants.Hardware.LIME_LIGHT);
        limelight.start();
    }

    public Limelight3A getLimelight(){
        return limelight;
    }
    public LLResult getLLResults(){
        return limelight.getLatestResult();
    }
    public List<LLResultTypes.FiducialResult> fiducialResult(){
        return limelight.getLatestResult().getFiducialResults();
    }
    public int getAprilTagID(){
        List<LLResultTypes.FiducialResult> fiducials = limelight.getLatestResult().getFiducialResults();
        int id = 0;
        for(LLResultTypes.FiducialResult fiducial : fiducials){
            id = fiducial.getFiducialId(); // The ID number of the fiducial
        }
        return id;
    }
}
