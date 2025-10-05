package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class RevUpToShoot extends CommandBase {
    ShooterSubsystem shooter;
    Timing.Timer timer;
    public RevUpToShoot(ShooterSubsystem shooter){
        this.shooter = shooter;
        addRequirements(shooter);
        timer = new Timing.Timer(RobotConstants.Teleop.SHOOTER_TIMER);
    }
    @Override
    public void initialize() {
        super.initialize();
        shooter.rampToSpeed(RobotConstants.Teleop.SHOOTER_SPEED);
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }
}
