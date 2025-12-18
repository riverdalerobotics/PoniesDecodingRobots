package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;

public class ChassisAutoMoveUsingTime extends CommandBase {
    ChassisSubsystem chassis;
    TelemetryManager telemetry;
    double[] speed;
    /**
     * Moves the robot forward
     * @param chassis the chassis subsystem
     * @param telemetry the telemetry manager
     * @param x the x speed that the chassis will move
     * @param y the y speed that the chassis will move
     * @param turn the speed at which the chassis will spin
     * */
    public ChassisAutoMoveUsingTime(ChassisSubsystem chassis, TelemetryManager telemetry, double x, double y, double turn){
        this.chassis = chassis;
        this.telemetry = telemetry;
        speed = new double[]{x, y, turn};
        addRequirements(chassis);
    }

    @Override
    public void execute() {
        super.execute();
        chassis.driveRobotOriented(speed[0],speed[1],speed[2]);
        telemetry.addData("Chassis is moving at: ", speed);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        chassis.driveRobotOriented(0,0,0);
    }
}
