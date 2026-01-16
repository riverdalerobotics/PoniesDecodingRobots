package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class IntakeCommand extends CommandBase {
    IntakeSubsystem intake;
    ShooterSubsystem snap, crackle, pop;
    public IntakeCommand(IntakeSubsystem intake, ShooterSubsystem snap, ShooterSubsystem crackle, ShooterSubsystem pop){
        this.intake = intake;
        this.snap=snap;
        this.crackle =crackle;
        this.pop=pop;
        addRequirements(intake, snap, crackle, pop);
    }

    @Override
    public void initialize() {
        super.initialize();
        snap.setHoodAngle(RobotConstants.Tuning.MAX_ANGLE);
        crackle.setHoodAngle(RobotConstants.Tuning.MAX_ANGLE);
        pop.setHoodAngle(RobotConstants.Tuning.MAX_ANGLE);
    }

    @Override
    public void execute() {
        super.execute();
        intake.spinIntake(RobotConstants.Teleop.INTAKE_SPEED);
        crackle.getShootMotor().set(RobotConstants.Teleop.SHOOTER_INTAKE_SPEED);
//        crackle.getShootMotor().set(-RobotConstants.Teleop.SHOOTER_INTAKE_SPEED);
//        pop.getShootMotor().set(-RobotConstants.Teleop.SHOOTER_INTAKE_SPEED);

    }

}
