package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class FeedShooter extends CommandBase {
    ShooterSubsystem shooter;

    public FeedShooter(ShooterSubsystem shooter){
        this.shooter = shooter;

        addRequirements(shooter);

    }

    @Override
    public void initialize() {
        super.initialize();
        shooter.feedShoot();

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
