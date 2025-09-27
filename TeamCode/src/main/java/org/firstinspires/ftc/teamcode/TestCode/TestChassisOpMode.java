package org.firstinspires.ftc.teamcode.TestCode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ChassisDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.DriveToLaunchZone;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;

@TeleOp(group = "Test", name = "Test op mode")
public class TestChassisOpMode extends CommandOpMode {
    ChassisSubsystem chassis;
    GamepadEx driver = new GamepadEx(gamepad1);
    TelemetryManager telemetryM;
    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        chassis = new ChassisSubsystem(hardwareMap, telemetryM);
        chassis.setDefaultCommand(new ChassisDefaultCommand(chassis, telemetryM, driver));
    }

    @Override
    public void run() {
        super.run();
        Button goToLaunchZone = new GamepadButton(
                driver, GamepadKeys.Button.A
        ).whenPressed(
                new DriveToLaunchZone('b', chassis)
        );
        telemetryM.update();
    }
}
