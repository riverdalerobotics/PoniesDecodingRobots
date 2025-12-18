package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class FarShot extends CommandBase {
    ShooterSubsystem shooter;
    public FarShot(ShooterSubsystem shooter){
        this.shooter = shooter;
        addRequirements(shooter);
    }
    @Override
    public void initialize() {
        shooter.setHoodAngle(RobotConstants.Tuning.MIN_ANGLE);
    }

    @Override
    public void execute() {
        super.execute();
        shooter.rampToSpeed(RobotConstants.Teleop.FAR_SHOT);
    }
}
