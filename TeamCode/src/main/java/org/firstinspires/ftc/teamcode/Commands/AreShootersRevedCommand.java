package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class AreShootersRevedCommand extends CommandBase {
    ShooterSubsystem snap, crackle, pop;
    boolean atPos = false;
    public AreShootersRevedCommand(ShooterSubsystem snap, ShooterSubsystem crackle, ShooterSubsystem pop){
        this.snap = snap;
        this.crackle = crackle;
        this.pop = pop;
    }

    @Override
    public void execute() {
        super.execute();
        atPos = snap.getShooterPID().atSetPoint() && crackle.getShooterPID().atSetPoint() && pop.getShooterPID().atSetPoint();
        snap.getTelemetry().addData("AT POS?", atPos);
    }

    @Override
    public boolean isFinished() {
        return atPos;
    }
}
