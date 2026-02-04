package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class Shoot extends SequentialCommandGroup {
    public Shoot(ShooterSubsystem snap, ShooterSubsystem crackle, ShooterSubsystem pop, char[] motif, boolean shootMotif){
        addRequirements(snap, crackle, pop);
        if(shootMotif){
            addCommands(new ShootMotif(snap, crackle, pop, motif));
        }
    }
}
