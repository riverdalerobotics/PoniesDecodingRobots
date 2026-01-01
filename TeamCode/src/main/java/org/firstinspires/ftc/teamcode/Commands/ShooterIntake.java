package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class ShooterIntake extends CommandBase {
    ShooterSubsystem shooter;
    public ShooterIntake(ShooterSubsystem shooter){
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
        shooter.setHoodAngle(RobotConstants.Tuning.MAX_ANGLE);
        shooter.rampToSpeed(RobotConstants.Teleop.SHOOTER_INTAKE_SPEED);
    }

    @Override
    public void execute() {
        super.execute();
        shooter.rampToSpeed(RobotConstants.Teleop.SHOOTER_INTAKE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        shooter.rampToSpeed(0);
    }
}
