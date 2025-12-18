package org.firstinspires.ftc.teamcode.Commands.Motifs;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.ShootSequence;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class ShootPGP extends SequentialCommandGroup {
    /**
     * Shoots the motif Purple Green Purple using all tree shooters
     * @param purpleOne one of the two shooters with a purple artifact
     * @param purpleTwo the other shooter with a purple artifact
     * @param green the shooter with a green artifact
     * */
    public ShootPGP(ShooterSubsystem purpleOne, ShooterSubsystem purpleTwo, ShooterSubsystem green){
        addCommands(new ShootSequence(purpleTwo), new ShootSequence(green), new ShootSequence(purpleOne));
        addRequirements(purpleOne, purpleTwo, green);
    }
}
