package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LLsubsystem;

public class ChassisLLAutoTurn extends CommandBase {
    PIDController hPID;
    LLsubsystem limelight;
    ChassisSubsystem chassis;
    Telemetry telemetry;

    double xSpeed, ySpeed, hSpeed, target, speed;
    public ChassisLLAutoTurn(ChassisSubsystem chassis, LLsubsystem limelight, Telemetry telemetry, double target){
        addRequirements(chassis);
        this.chassis = chassis;
        this.limelight = limelight;
        this.target = target;
        this.telemetry = telemetry;

        hPID = new PIDController(RobotConstants.Tuning.CHASSIS_TURN_PID_COEFFICIENTS[0],
                RobotConstants.Tuning.CHASSIS_TURN_PID_COEFFICIENTS[1], RobotConstants.Tuning.CHASSIS_TURN_PID_COEFFICIENTS[2]);
    }

    @Override
    public void initialize() {
        super.initialize();

        hPID.setSetPoint(target);

        hPID.setTolerance(RobotConstants.Tuning.CHASSIS_TOLERANCE[1]);
    }

    @Override
    public void execute() {
        super.execute();

            speed = hPID.calculate(chassis.yawPitchRollAngles().getYaw(AngleUnit.DEGREES), target);
             hSpeed = RobotConstants.clamp(speed, -1, 1);
            telemetry.addData("calcSpeed", speed);
        chassis.driveFieldOriented(0, 0, hSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        chassis.driveRobotOriented(0,0,0);
    }

    @Override
    public boolean isFinished() {
        return hPID.atSetPoint();
    }
}
