package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class IntakeFeeding extends CommandBase {
    ShooterSubsystem shooter;
    /**
     * Moves the shooter feed servo to feed the ball
     * @param shooter The shooter that will be fed
     * */
    public IntakeFeeding(ShooterSubsystem shooter){
        this.shooter = shooter;

    }

    @Override
    public void initialize() {
        super.initialize();
        shooter.intakeFeeder();

    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        shooter.resetFeed();
    }

    }
