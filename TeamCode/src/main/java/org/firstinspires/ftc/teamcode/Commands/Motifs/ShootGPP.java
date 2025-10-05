package org.firstinspires.ftc.teamcode.Commands.Motifs;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.ShootSequence;
import org.firstinspires.ftc.teamcode.Commands.ShootTwo;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class ShootGPP extends SequentialCommandGroup {
    public ShootGPP(ShooterSubsystem purpleOne, ShooterSubsystem purpleTwo, ShooterSubsystem green){
        addCommands(new ShootSequence(green), new ShootTwo(purpleOne, purpleTwo));
    }
}
