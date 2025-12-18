package org.firstinspires.ftc.teamcode.Commands.Motifs;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.ShootSequence;
import org.firstinspires.ftc.teamcode.Commands.ShootTwo;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class ShootPPG extends SequentialCommandGroup {
    /**
     * Shoots the motif Purple Purple Green using all tree shooters
     * @param purpleOne one of the two shooters with a purple artifact
     * @param purpleTwo the other shooter with a purple artifact
     * @param green the shooter with a green artifact
     * */
    public ShootPPG(ShooterSubsystem purpleOne, ShooterSubsystem purpleTwo, ShooterSubsystem green){
        addCommands(new ShootTwo(purpleOne, purpleTwo), new ShootSequence(green));
        addRequirements(purpleOne, purpleTwo, green);
    }
}
