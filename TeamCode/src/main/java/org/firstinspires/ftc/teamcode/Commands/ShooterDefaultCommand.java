package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LLsubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class ShooterDefaultCommand extends CommandBase{
    ShooterSubsystem shooter;
    LLResult limelight;
    public ShooterDefaultCommand(ShooterSubsystem shooter){
        this.shooter = shooter;
        limelight = shooter.getLLResult();
    }

    @Override
    public void initialize() {
        super.initialize();
        shooter.rampToSpeed(0);
        shooter.resetFeed();
    }

    @Override
    public void execute() {
        super.execute();
        if(limelight.isValid() && limelight != null){
            shooter.setHoodAngle(RobotConstants.clamp(limelight.getTa()*RobotConstants.Tuning.TA_TO_ANGLE, 0, 50));
        }else{
            shooter.setHoodAngle(0);
        }

    }
}
