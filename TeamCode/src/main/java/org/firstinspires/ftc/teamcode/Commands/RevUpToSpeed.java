package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class RevUpToSpeed extends CommandBase {
    ShooterSubsystem shooter;
    Timing.Timer timer;
    public RevUpToSpeed(ShooterSubsystem shooter){
        this.shooter = shooter;
        addRequirements(shooter);
        timer = new Timing.Timer(RobotConstants.Teleop.SHOOTER_TIMER);
    }
    @Override
    public void initialize() {
        super.initialize();

        timer.start();
    }

    @Override
    public void execute() {
        super.execute();
        if(shooter.getLLResult().isValid()){
            if(shooter.getLLResult().getTa()<RobotConstants.Teleop.CLOSE_SHOT_THRESHOLD){
                RobotConstants.Hardware.SHOOTER_WHEEL_GEAR_RATIO = RobotConstants.Teleop.FAR_SHOT;
            }else{
                RobotConstants.Hardware.SHOOTER_WHEEL_GEAR_RATIO = RobotConstants.Teleop.CLOSE_SHOT_TELEOP;
            }

            shooter.setHoodAngle(RobotConstants.clamp(RobotConstants.Tuning.TA_TO_ANGLE*shooter.getLLResult().getTa(), -0.05, 0.16));
        }else{
            shooter.setHoodAngle(RobotConstants.Tuning.MAX_ANGLE);
            RobotConstants.Hardware.SHOOTER_WHEEL_GEAR_RATIO = RobotConstants.Teleop.VERY_CLOSE_SHOT;
        }
        shooter.rampToSpeed(RobotConstants.Hardware.SHOOTER_WHEEL_GEAR_RATIO);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
