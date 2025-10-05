package org.firstinspires.ftc.teamcode.Commands.Motifs;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.ShootSequence;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class ShootPGP extends SequentialCommandGroup {
    public ShootPGP(ShooterSubsystem purpleOne, ShooterSubsystem purpleTwo, ShooterSubsystem green){
        addCommands(new ShootSequence(purpleTwo), new ShootSequence(green), new ShootSequence(purpleOne));
        addRequirements(purpleOne, purpleTwo, green);
    }
}
