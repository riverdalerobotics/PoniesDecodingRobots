package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;


import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

import java.util.ArrayList;

public class ShootMotif extends SequentialCommandGroup {
    public ShootMotif(ShooterSubsystem snap, ShooterSubsystem crackle, ShooterSubsystem pop, char[] motif){
        char[] gpp = {'g', 'p', 'p'};
        char[] ppg = {'p', 'p', 'g'};

        ArrayList<ShooterSubsystem> shooters = new ArrayList<>();
        shooters.add(snap);
        shooters.add(crackle);
        shooters.add(pop);
        ArrayList<ShooterSubsystem> orderedShooters = new ArrayList<>();
        orderedShooters.add(snap);
        orderedShooters.add(crackle);
        orderedShooters.add(pop);


        for(char motifSpot : ppg){
            shooters.forEach(shooter -> {
                if(motifSpot == shooter.colour){
                    orderedShooters.remove(shooter);
                    orderedShooters.add(shooter);
                }
            });
        }

        addRequirements(snap, crackle, pop);

    }
}
