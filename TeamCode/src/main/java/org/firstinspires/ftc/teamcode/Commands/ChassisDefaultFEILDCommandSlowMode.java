package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;

public class ChassisDefaultFEILDCommandSlowMode extends CommandBase {
    ChassisSubsystem chassis;
    GamepadEx gamepad;
    /**
     * Chassis default command that makes it the chassis drives using field oriented
     * */
    public ChassisDefaultFEILDCommandSlowMode(ChassisSubsystem chassis, GamepadEx gamepad){
        this.chassis = chassis;
        this.gamepad = gamepad;
        addRequirements(chassis);
    }

    @Override
    public void execute() {
        super.execute();
        chassis.fieldOriented(-gamepad.getLeftX()*0.75, -gamepad.getLeftY()*0.75, -gamepad.getRightX()*0.75);


    }
}
