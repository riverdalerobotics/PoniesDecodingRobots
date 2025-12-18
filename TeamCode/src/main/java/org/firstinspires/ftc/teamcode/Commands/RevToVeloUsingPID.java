package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class RevToVeloUsingPID extends CommandBase {
    double setpoint;
    ShooterSubsystem shooter;
    PIDController shooterPID;
    public RevToVeloUsingPID(ShooterSubsystem shooter, double velocity){
        this.shooter = shooter;
        this.setpoint = velocity;
        this.shooterPID = new PIDController(RobotConstants.Tuning.SHOOTER_PID_COEFFICIENTS[0],
                RobotConstants.Tuning.SHOOTER_PID_COEFFICIENTS[1],
                RobotConstants.Tuning.SHOOTER_PID_COEFFICIENTS[2]);

    }
    @Override
    public void initialize() {
        shooterPID.setSetPoint(setpoint);
    }

    @Override
    public void execute() {
        double speed = shooterPID.calculate(RobotConstants.clamp(shooter.getSpeed(),-1,1));
        shooter.rampToSpeed(speed);
    }
}
