package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;

public class ChassisDefaultCommand extends CommandBase {
    ChassisSubsystem chassis;
    PanelsTelemetry telemetry;
    GamepadEx gamepad;
    public ChassisDefaultCommand(ChassisSubsystem chassis, PanelsTelemetry telemetry, GamepadEx gamepad){
        this.chassis = chassis;
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        addRequirements(chassis);
    }

    @Override
    public void execute() {
        super.execute();
        chassis.driveFieldOriented(gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX());
    }
}
