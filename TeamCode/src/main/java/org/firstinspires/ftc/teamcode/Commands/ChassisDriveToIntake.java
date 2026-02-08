package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;

public class ChassisDriveToIntake extends CommandBase {
    ChassisSubsystem chassis;
    PIDController xPid;
    PIDController yPid;
    public ChassisDriveToIntake(ChassisSubsystem chassis){
        this.chassis = chassis;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        super.initialize();
        xPid = new PIDController(RobotConstants.Tuning.CHASSIS_PID_COEFFICIENTS_POINT_AUTO[0], RobotConstants.Tuning.CHASSIS_PID_COEFFICIENTS_POINT_AUTO[1], RobotConstants.Tuning.CHASSIS_PID_COEFFICIENTS_POINT_AUTO[2]);
        yPid = new PIDController(RobotConstants.Tuning.CHASSIS_PID_COEFFICIENTS_POINT_AUTO[0], RobotConstants.Tuning.CHASSIS_PID_COEFFICIENTS_POINT_AUTO[1], RobotConstants.Tuning.CHASSIS_PID_COEFFICIENTS_POINT_AUTO[2]);
        xPid.setSetPoint(RobotConstants.Teleop.INTAKE_X_ANGLE_CHASSIS);
        yPid.setSetPoint(RobotConstants.Teleop.INTAKE_Y_ANGLE_CHASSIS);
        xPid.setTolerance(RobotConstants.Tuning.CHASSIS_TOLERANCE[1]);
        yPid.setTolerance(RobotConstants.Tuning.CHASSIS_TOLERANCE[1]);

//        chassis.selectPipeline(RobotConstants.Hardware.INTAKE_PIPELINE);
    }

    @Override
    public void execute() {
        super.execute();
        double xSpeed = RobotConstants.clamp(xPid.calculate(chassis.getLLresults().getTx()), -1, 1);
        double ySpeed = RobotConstants.clamp(yPid.calculate(chassis.getLLresults().getTy()), -1, 1);
        chassis.driveFieldOriented(xSpeed, ySpeed, 0);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
//        chassis.selectPipeline(RobotConstants.Hardware.APRIL_TAG_PIPELINE);
        xPid.reset();
        yPid.reset();
    }

    @Override
    public boolean isFinished() {
        return xPid.atSetPoint() && yPid.atSetPoint();
    }
}
