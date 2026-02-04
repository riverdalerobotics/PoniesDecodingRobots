package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.LLsubsystem;

public class LimelightSeesAT extends CommandBase {
    LLsubsystem limelight;
    public LimelightSeesAT(LLsubsystem limelight){
        this.limelight = limelight;
    }

    @Override
    public boolean isFinished() {
        return limelight.getLLResults().isValid();
    }
}

