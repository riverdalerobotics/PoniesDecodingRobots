package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

import java.util.Collections;
import java.util.Set;

public class CloseShot extends CommandBase {
    ShooterSubsystem shooter;
    public CloseShot(ShooterSubsystem shooter){
        this.shooter = shooter;
        addRequirements(shooter);
    }
    @Override
    public void initialize() {
        shooter.setHoodAngle(RobotConstants.Tuning.MAX_ANGLE);
        RobotConstants.Hardware.SHOOTER_WHEEL_GEAR_RATIO = RobotConstants.Teleop.VERY_CLOSE_SHOT;
    }

    @Override
    public void execute() {
        super.execute();

        shooter.rampToSpeed(RobotConstants.Teleop.VERY_CLOSE_SHOT);
    }
}
