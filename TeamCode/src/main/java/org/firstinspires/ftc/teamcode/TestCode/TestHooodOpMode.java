package org.firstinspires.ftc.teamcode.TestCode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.gamepad.Gamepad;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.ShooterDefaultCommand;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@TeleOp(group = "Test", name = "Hood Test")
public class TestHooodOpMode extends CommandOpMode {
    ShooterSubsystem snap, crackle, pop;
    ShooterDefaultCommand snapDefault, crackleDefault, popDefault;
    GamepadEx operator = new GamepadEx(gamepad2);
    TelemetryManager telemetryM;

    @Override
    public void initialize(){
        snap = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.SNAP);
        crackle = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.CRACKLE);
        pop = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.POP);
        snapDefault = new ShooterDefaultCommand(snap);
        crackleDefault = new ShooterDefaultCommand(crackle);
        popDefault = new ShooterDefaultCommand(pop);

        register(snap, crackle, pop);
        schedule(snapDefault, crackleDefault, popDefault);

        snap.setDefaultCommand(snapDefault);
        crackle.setDefaultCommand(crackleDefault);
        pop.setDefaultCommand(popDefault);
    }

    @Override
    public void run(){

    }
}
