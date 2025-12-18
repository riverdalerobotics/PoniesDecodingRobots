package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class FeedShooter extends CommandBase {
    ShooterSubsystem shooter;
    /**
     * Moves the shooter feed servo to feed the ball
     * @param shooter The shooter that will be fed
     * */
    public FeedShooter(ShooterSubsystem shooter){
        this.shooter = shooter;

    }

    @Override
    public void initialize() {
        super.initialize();
        shooter.feedShoot();

    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        shooter.resetFeed();
    }

    }
