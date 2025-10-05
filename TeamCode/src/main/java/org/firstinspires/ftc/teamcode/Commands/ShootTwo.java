package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class ShootTwo extends ParallelCommandGroup {
    public ShootTwo(ShooterSubsystem shooterOne, ShooterSubsystem shooterTwo){
        addCommands(new ShootSequence(shooterOne), new ShootSequence(shooterTwo));
        addRequirements(shooterTwo, shooterOne);
    }
}
