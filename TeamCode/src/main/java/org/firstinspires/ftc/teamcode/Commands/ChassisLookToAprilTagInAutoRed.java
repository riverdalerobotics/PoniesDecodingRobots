package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LLsubsystem;

public class ChassisLookToAprilTagInAutoRed extends CommandBase {
    ChassisSubsystem chassisSubsystem;
    LLsubsystem limer;
    PIDController hPID;
    TelemetryManager telemetryManager;
    double targetAngle;
    /**
     * Makes the chassis point to any april tag using a PID controller, ends when the chassis is
     * within CHASSIS_TOLERANCE -> see Robot constants for more
     * */
    public ChassisLookToAprilTagInAutoRed(ChassisSubsystem cSubsystem, LLsubsystem limelight, TelemetryManager telemetryManager, double targetAngle){
        chassisSubsystem = cSubsystem;
        this.telemetryManager = telemetryManager;
        limer = limelight;
        this.targetAngle = targetAngle;
        this.hPID = new PIDController(RobotConstants.Tuning.CHASSIS_PID_COEFFICIENTS_POINT_AUTO[0], RobotConstants.Tuning.CHASSIS_PID_COEFFICIENTS_POINT_AUTO[1], RobotConstants.Tuning.CHASSIS_PID_COEFFICIENTS_POINT_AUTO[2]);
        addRequirements(cSubsystem);
    }

    @Override
    public void initialize(){
        super.initialize();
        hPID.setTolerance(RobotConstants.Tuning.CHASSIS_TOLERANCE[1]);
        hPID.setSetPoint(RobotConstants.Tuning.POINT_AT_AT_TARGET_AUTO);
    }

    @Override
    public void execute(){
        super.execute();
        if(limer.getLLResults().isValid()) {
            double speed = RobotConstants.clamp(hPID.calculate(limer.getLLResults().getTx()), -1, 1);
            telemetryManager.addData("Chassis h Speed", speed);
            chassisSubsystem.driveRobotOriented(0, 0, speed);
        }else{
            chassisSubsystem.driveRobotOriented(0,0, 0.1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        hPID.reset();
    }

    @Override
    public boolean isFinished() {
        return hPID.atSetPoint();
    }
}
