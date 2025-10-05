package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class ShootSequence extends SequentialCommandGroup {
    public ShootSequence(ShooterSubsystem shooter){
        addCommands(new FeedShooter(shooter), new RevUpToShoot(shooter));
        addRequirements(shooter);
    }
}
