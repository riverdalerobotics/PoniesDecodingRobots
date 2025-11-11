package org.firstinspires.ftc.teamcode.TestCode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ChassisDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.ChassisLookToAprilTag;
import org.firstinspires.ftc.teamcode.Commands.DriveToLaunchZone;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LLsubsystem;

@TeleOp(group = "Test", name = "Test Chassis op mode")
public class TestChassisOpMode extends CommandOpMode {
    ChassisSubsystem chassis;
    GamepadEx driver;
    TelemetryManager telemetryM;
    ChassisDefaultCommand chassisDefaultCommand;
    LLsubsystem limelight;

    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        limelight = new LLsubsystem(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        chassis = new ChassisSubsystem(hardwareMap, telemetryM);
        chassisDefaultCommand = new ChassisDefaultCommand(chassis, telemetryM, driver);
        register(chassis);
        schedule(chassisDefaultCommand);
        chassis.setDefaultCommand(chassisDefaultCommand);

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        super.run();
        Button pointAtAT = new GamepadButton(
                driver, GamepadKeys.Button.A
        ).whenPressed(
                new ChassisLookToAprilTag(chassis, limelight, telemetryM, 0, driver)
        );
        telemetryM.update();
    }
}
