package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class LLsubsystem {
    Limelight3A limelight;
    public LLsubsystem(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, RobotConstants.Hardware.LIME_LIGHT);
    }

    public Limelight3A getLimelight(){
        return limelight;
    }
}
