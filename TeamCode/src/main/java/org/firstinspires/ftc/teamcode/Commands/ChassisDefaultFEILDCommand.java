package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;

public class ChassisDefaultFEILDCommand extends CommandBase {
    ChassisSubsystem chassis;
    TelemetryManager telemetry;
    GamepadEx gamepad;
    public ChassisDefaultFEILDCommand(ChassisSubsystem chassis, TelemetryManager telemetry, GamepadEx gamepad){
        this.chassis = chassis;
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        addRequirements(chassis);
    }

    @Override
    public void execute() {
        super.execute();
        chassis.driveFieldOriented(gamepad.getLeftX(), -gamepad.getLeftY(), gamepad.getRightX());
        telemetry.addLine("Chassis default is running");

    }
}
