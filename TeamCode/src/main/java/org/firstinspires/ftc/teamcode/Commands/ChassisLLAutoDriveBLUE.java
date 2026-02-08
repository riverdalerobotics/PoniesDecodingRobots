package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LLsubsystem;

public class ChassisLLAutoDriveBLUE extends CommandBase {
    PIDController xPID, yPID, hPID;
    Pose target;
    LLsubsystem limelight;
    ChassisSubsystem chassis;
    Telemetry telemetry;
    double xSpeed, ySpeed, hSpeed;
    public ChassisLLAutoDriveBLUE(ChassisSubsystem chassis, LLsubsystem limelight, Telemetry telemetry, Pose target){
        addRequirements(chassis);
        this.chassis = chassis;
        this.limelight = limelight;
        this.target = target;
        this.telemetry = telemetry;

        xPID = new PIDController(-RobotConstants.Tuning.CHASSIS_DRIVE_PID_COEFFICIENTS[0],
                -RobotConstants.Tuning.CHASSIS_DRIVE_PID_COEFFICIENTS[1], -RobotConstants.Tuning.CHASSIS_DRIVE_PID_COEFFICIENTS[2]);
        yPID = new PIDController(-RobotConstants.Tuning.CHASSIS_DRIVE_PID_COEFFICIENTS[0],
                -RobotConstants.Tuning.CHASSIS_DRIVE_PID_COEFFICIENTS[1], -RobotConstants.Tuning.CHASSIS_DRIVE_PID_COEFFICIENTS[2]);
        hPID = new PIDController(RobotConstants.Tuning.CHASSIS_TURN_PID_COEFFICIENTS[0],
                RobotConstants.Tuning.CHASSIS_TURN_PID_COEFFICIENTS[1], RobotConstants.Tuning.CHASSIS_TURN_PID_COEFFICIENTS[2]);    }

    @Override
    public void initialize() {
        super.initialize();
        xPID.setSetPoint(target.getX());
        yPID.setSetPoint(target.getY());
        hPID.setSetPoint(target.getHeading());
        xPID.setTolerance(RobotConstants.Tuning.CHASSIS_TOLERANCE[0]);
        yPID.setTolerance(RobotConstants.Tuning.CHASSIS_TOLERANCE[0]);
        hPID.setTolerance(RobotConstants.Tuning.CHASSIS_TOLERANCE[1]);

    }

    @Override
    public void execute() {
        super.execute();

        limelight.getLimelight().updateRobotOrientation(chassis.yawPitchRollAngles().getYaw());
        if(limelight.getLLResults().isValid()) {
             xSpeed = RobotConstants.clamp(xPID.calculate(limelight.getLLResults().getBotpose_MT2().getPosition().x), -1, 1);
             ySpeed = RobotConstants.clamp(yPID.calculate(limelight.getLLResults().getBotpose_MT2().getPosition().y), -1, 1);
             hSpeed = RobotConstants.clamp(hPID.calculate(chassis.yawPitchRollAngles().getYaw(AngleUnit.DEGREES)), -1, 1);
        }else{
            //TODO: you can edit these two lines out if it aint buggy as shit
            xSpeed = 0;//1
            ySpeed = 0;//2
             hSpeed = RobotConstants.clamp(hPID.calculate(chassis.yawPitchRollAngles().getYaw(AngleUnit.DEGREES)), -1, 1);
        }
        chassis.driveFieldOriented(xSpeed, ySpeed, hSpeed);
    }

    @Override
    public boolean isFinished() {
        return xPID.atSetPoint()&&yPID.atSetPoint()&&hPID.atSetPoint();
    }
}
