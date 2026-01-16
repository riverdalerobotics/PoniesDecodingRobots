package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class RevToVeloUsingPID extends CommandBase {
    double setpoint;
    ShooterSubsystem shooter;
    Telemetry telemetry;
    PIDFController shooterPID;
    public RevToVeloUsingPID(ShooterSubsystem shooter, Telemetry telemetry){
        this.shooter = shooter;
        addRequirements(shooter);
        this.telemetry = telemetry;
        this.shooterPID = shooter.getShooterPID();
    }
    @Override
    public void initialize() {
        shooter.getShootMotor().motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterPID.setTolerance(RobotConstants.Tuning.SHOOTER_TOLERANCE);
        shooterPID.setSetPoint(setpoint);
    }

    @Override
    public void execute() {
        if(shooter.getLLResult().isValid()){
            if(shooter.getLLResult().getTa()<RobotConstants.Teleop.CLOSE_SHOT_THRESHOLD){
                setpoint = RobotConstants.Teleop.FAR_SHOT;
            }else{
                setpoint = RobotConstants.Teleop.CLOSE_SHOT;
            }

            shooter.setHoodAngle(RobotConstants.clamp(RobotConstants.Tuning.TA_TO_ANGLE*shooter.getLLResult().getTa(), -0.05, 0.16));
        }
        shooterPID.setPIDF(RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[0],
                RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[1],
                RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[2],
                RobotConstants.Tuning.SHOOTER_PIDF_COEFFICIENTS[3]);
        double speed = RobotConstants.clamp(shooterPID.calculate(shooter.getSpeed(), setpoint),0,1);
        telemetry.addData("set Speed", speed);
        telemetry.addData("Ta", shooter.getLLResult().getTa());
        telemetry.addData("The THINGYMAGIC", shooterPID.calculate(shooter.getSpeed()));
        telemetry.addData("current Speed", shooter.getSpeed());
        shooter.setSpeed = speed;
        telemetry.addData("Setpoint", setpoint);
        telemetry.update();
        shooter.getShootMotor().set(speed);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        shooterPID.reset();
        //penis
    }

//    @Override
//    public boolean isFinished() {
//        return shooterPID.atSetPoint();
//    }
}
