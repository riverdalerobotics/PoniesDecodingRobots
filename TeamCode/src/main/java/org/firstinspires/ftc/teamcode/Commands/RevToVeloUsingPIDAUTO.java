package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class RevToVeloUsingPIDAUTO extends CommandBase {
    double setpoint;
    ShooterSubsystem shooter;
    Telemetry telemetry;
    PIDFController shooterPID;
    boolean stop = false;
    public RevToVeloUsingPIDAUTO(ShooterSubsystem shooter, Telemetry telemetry){
        this.shooter = shooter;
        addRequirements(shooter);
        this.telemetry = telemetry;
        this.shooterPID = shooter.getShooterPID();
    }
    public RevToVeloUsingPIDAUTO(ShooterSubsystem shooter, Telemetry telemetry, boolean stop){
        this.shooter = shooter;
        addRequirements(shooter);
        this.telemetry = telemetry;
        this.shooterPID = shooter.getShooterPID();
        this.stop = stop;
    }
    @Override
    public void initialize() {
        setpoint = RobotConstants.Teleop.CLOSE_SHOT_TELEOP;
        shooter.getShootMotor().motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterPID.setSetPoint(setpoint);
    }

    @Override
    public void execute() {
        shooterPID.setSetPoint(setpoint);
        if(shooter.getLLResult().isValid()){
            if(shooter.getLLResult().getTa()<RobotConstants.Teleop.CLOSE_SHOT_THRESHOLD){
                setpoint = RobotConstants.Teleop.FAR_SHOT;
            }else{
                setpoint = RobotConstants.Teleop.CLOSE_SHOT_AUTO;
            }
            shooter.setHoodAngle(RobotConstants.clamp(RobotConstants.Tuning.TA_TO_ANGLE*shooter.getLLResult().getTa(), -0.05, 0.16));
        }
        shooterPID.setPIDF(RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[0],
                RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[1],
                RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[2],
                RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[3]);
        double speed = RobotConstants.clamp(shooterPID.calculate(shooter.getSpeed(), setpoint),0,1);
        shooter.getTelemetry().addData("Calc Speed", speed);
        telemetry.update();
        shooter.getShootMotor().set(speed);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        shooterPID.reset();
        if(stop){
            shooter.getShootMotor().set(0);
        }
    }

}
