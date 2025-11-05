package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LLsubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class ShooterDefaultCommand extends CommandBase{
    ShooterSubsystem shooter;
    Gamepad op;
    public ShooterDefaultCommand(ShooterSubsystem shooter, Gamepad op){
        this.shooter = shooter;
        this.op = op;
        addRequirements(shooter);

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
        if(op.a){
            shooter.setHoodAngle(0);
        }
        if(op.b){
            shooter.setHoodAngle(0.25);
        }
        if(op.x){
            shooter.setHoodAngle(0.5);
        }
        shooter.setHoodAngle(0);
        shooter.setHoodAngle(op.left_stick_y);


    }
}
