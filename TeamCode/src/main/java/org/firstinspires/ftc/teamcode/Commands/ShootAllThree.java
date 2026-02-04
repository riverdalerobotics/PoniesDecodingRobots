package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class ShootAllThree extends SequentialCommandGroup {
    public ShootAllThree(ShooterSubsystem snap, ShooterSubsystem crackle, ShooterSubsystem pop, Telemetry telemetry){
       addCommands(new ParallelDeadlineGroup(
                       new WaitUntilCommand(()->snap.getShooterPID().atSetPoint()&&pop.getShooterPID().atSetPoint()&&crackle.getShooterPID().atSetPoint()),
                       new RevToVeloUsingPID(snap, telemetry),
                       new RevToVeloUsingPID(crackle, telemetry),
                       new RevToVeloUsingPID(pop, telemetry)
               ),
               new ParallelRaceGroup(
                       new RevToVeloUsingPID(snap, telemetry, true),
                       new RevToVeloUsingPID(crackle, telemetry, true),
                       new RevToVeloUsingPID(pop, telemetry, true),
                       new ParallelDeadlineGroup(
                               new Timer(RobotConstants.Teleop.HOLD_THE_ARM),
                               new FeedShooter(snap)),
                       new ParallelDeadlineGroup(
                               new Timer(RobotConstants.Teleop.HOLD_THE_ARM),
                               new FeedShooter(crackle)),
                       new ParallelDeadlineGroup(
                               new Timer(RobotConstants.Teleop.HOLD_THE_ARM),
                               new FeedShooter(pop))
               )
       );
    }

}
