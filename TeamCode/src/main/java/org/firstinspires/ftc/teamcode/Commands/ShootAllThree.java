package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class ShootAllThree extends ParallelCommandGroup{
    public ShootAllThree(ShooterSubsystem shooterOne, ShooterSubsystem shooterTwo, ShooterSubsystem shooterThree){
        addRequirements(shooterOne, shooterTwo, shooterThree);
        addCommands(new ShootSequence(shooterOne), new ShootSequence(shooterTwo), new ShootSequence(shooterThree));
    }
}
