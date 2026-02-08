package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LLsubsystem;

public class ChassisLookToAprilTag extends CommandBase {
    ChassisSubsystem chassisSubsystem;
    LLsubsystem limer;
    PIDController hPID;
    TelemetryManager telemetryManager;
    double targetAngle;
    GamepadEx driver;
    /**
     * Makes the chassis point to any april tag using a PID controller, doesn't end
     * */
    public ChassisLookToAprilTag(ChassisSubsystem cSubsystem, LLsubsystem limelight, TelemetryManager telemetryManager, double targetAngle, GamepadEx driver){
        chassisSubsystem = cSubsystem;
        this.driver = driver;
        this.telemetryManager = telemetryManager;
        limer = limelight;
        this.targetAngle = targetAngle;
        this.hPID = new PIDController(RobotConstants.Tuning.CHASSIS_PID_COEFFICIENTS_POINT[0], RobotConstants.Tuning.CHASSIS_PID_COEFFICIENTS_POINT_AUTO[1], RobotConstants.Tuning.CHASSIS_PID_COEFFICIENTS_POINT_AUTO[2]);
        addRequirements(cSubsystem);
    }

    @Override
    public void initialize(){
        super.initialize();
        hPID.setTolerance(RobotConstants.Tuning.CHASSIS_TOLERANCE[1]);
        hPID.setSetPoint(RobotConstants.Tuning.POINT_AT_AT_TARGET);
    }

    @Override
    public void execute(){
        super.execute();
        if(limer.getLLResults().isValid()) {
            double speed = RobotConstants.clamp(hPID.calculate(limer.getLLResults().getTx()), -1, 1);
            telemetryManager.addData("Chassis h Speed", speed);
            chassisSubsystem.fieldOriented(-driver.getLeftX(), -driver.getLeftY(), -speed);
        }else{
            chassisSubsystem.fieldOriented(-driver.getLeftX(), -driver.getLeftY(), -driver.getRightX());
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        hPID.reset();
    }

}
