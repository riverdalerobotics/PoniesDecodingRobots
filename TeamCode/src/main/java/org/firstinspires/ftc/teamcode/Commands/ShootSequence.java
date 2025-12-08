package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class ShootSequence extends SequentialCommandGroup {
    public ShootSequence(ShooterSubsystem shooter){
        addCommands(new ParallelDeadlineGroup(new Timer(RobotConstants.Teleop.SHOOTER_TIMER), new RevUpToShoot(shooter)), new ParallelDeadlineGroup(new Timer(200L), new FeedShooter(shooter)));
        addRequirements(shooter);
    }
}
