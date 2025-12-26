package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class ShooterDefaultCommand extends CommandBase{
    ShooterSubsystem shooter;
    public ShooterDefaultCommand(ShooterSubsystem shooter){
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
        shooter.rampToSpeed(0);
        shooter.resetFeed();
        shooter.setHoodAngle(RobotConstants.Tuning.MAX_ANGLE);
    }

    @Override
    public void execute() {
        super.execute();
        shooter.getShootMotor().set(0);
        if(shooter.getLLResult().isValid()){
            if(shooter.getLLResult().getTa()<RobotConstants.Teleop.CLOSE_SHOT_THRESHOLD){
                RobotConstants.Hardware.SHOOTER_WHEEL_GEAR_RATIO = RobotConstants.Teleop.CLOSE_SHOT;
            }else{
                RobotConstants.Hardware.SHOOTER_WHEEL_GEAR_RATIO = RobotConstants.Teleop.FAR_SHOT;
            }

            shooter.setHoodAngle(RobotConstants.clamp(RobotConstants.Tuning.TA_TO_ANGLE*shooter.getLLResult().getTa(), RobotConstants.Tuning.MIN_ANGLE, RobotConstants.Tuning.MAX_ANGLE));
        }//else{
//            shooter.setHoodAngle(RobotConstants.Tuning.MAX_ANGLE);
//            RobotConstants.Hardware.SHOOTER_WHEEL_GEAR_RATIO = RobotConstants.Teleop.VERY_CLOSE_SHOT;
//        }


    }
}
