package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;

public class ChassisDefaultAuto extends CommandBase {
    ChassisSubsystem chassis;
    TelemetryManager telemetry;
    /**
     * Makes the chassis stay still for auto
     * */
    public ChassisDefaultAuto(ChassisSubsystem chassis, TelemetryManager telemetry){
        this.chassis = chassis;
        this.telemetry = telemetry;
        addRequirements(chassis);
    }

    @Override
    public void execute() {
        super.execute();
        chassis.driveRobotOriented(0,0,0);
        telemetry.addLine("Chassis auto default is running");

    }
}
