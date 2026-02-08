package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class SpitCommand extends CommandBase {
    IntakeSubsystem intake;
    public SpitCommand(IntakeSubsystem intake){
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();


    }

    @Override
    public void execute() {
        super.execute();
        intake.spinIntake(-RobotConstants.Teleop.INTAKE_SPEED/1.5);

//        crackle.getShootMotor().set(-RobotConstants.Teleop.SHOOTER_INTAKE_SPEED);
//        pop.getShootMotor().set(-RobotConstants.Teleop.SHOOTER_INTAKE_SPEED);

    }

    @Override
    public void end(boolean interrupted) {
        intake.spinIntake(0);
    }
}
