package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Motifs.ShootGPP;
import org.firstinspires.ftc.teamcode.Commands.Motifs.ShootPGP;
import org.firstinspires.ftc.teamcode.Commands.Motifs.ShootPPG;
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
            boolean hasBeenFound = false;
            for(ShooterSubsystem shooter: shooters){
                if(motifSpot == shooter.colour && !hasBeenFound){
                    orderedShooters.add(shooter);
                    shooters.remove(shooter);
                    hasBeenFound = true;
                }
            }
        }

        if(motif == gpp){
            addCommands(new ShootGPP(orderedShooters.get(0), orderedShooters.get(1), orderedShooters.get(2)));
        } else if (motif == ppg) {
            addCommands(new ShootPPG(orderedShooters.get(0), orderedShooters.get(1), orderedShooters.get(2)));
        }else{
            addCommands(new ShootPGP(orderedShooters.get(0), orderedShooters.get(1), orderedShooters.get(2)));
        }
        addRequirements(snap, crackle, pop);

    }
}
