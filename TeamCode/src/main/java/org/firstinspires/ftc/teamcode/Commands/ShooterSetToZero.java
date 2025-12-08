package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class ShooterSetToZero extends CommandBase {
    ShooterSubsystem shooter;
    GamepadEx op;
    public ShooterSetToZero(ShooterSubsystem shooter, GamepadEx op){
        this.shooter = shooter;
        this.op = op;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
        shooter.setHoodAngle(0.1);
    }

    @Override
    public void execute() {
    }
}