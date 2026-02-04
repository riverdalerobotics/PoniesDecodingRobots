package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class ShootSequence extends SequentialCommandGroup {
//    public ShootSequence(ShooterSubsystem shooter, Telemetry telemetry){
//        addCommands(new ParallelDeadlineGroup(
//                    new AreShootersRevedCommand(shooter),
//                    new RevToVeloUsingPID(shooter, telemetry)),
//                new ParallelDeadlineGroup(
//                        new Timer(RobotConstants.Teleop.HOLD_THE_ARM),
//                        new RevToVeloUsingPID(shooter, telemetry),
//                        new FeedShooter(shooter)));
//        addRequirements(shooter);
//    }
}
