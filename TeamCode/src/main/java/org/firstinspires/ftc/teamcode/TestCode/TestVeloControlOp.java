package org.firstinspires.ftc.teamcode.TestCode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.gamepad.Gamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ShooterDefaultCommand;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@TeleOp(name="test velo op", group = "Test")
public class TestVeloControlOp extends CommandOpMode {
    ShooterSubsystem snap;
    TelemetryManager telemetryM;
    GamepadEx gamepad;

    @Override
    public void initialize() {
        this.gamepad = new GamepadEx(gamepad1);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        snap = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.SNAP);
        snap.setDefaultCommand(new ShooterDefaultCommand(snap));
    }

    @Override
    public void run() {
        super.run();
        CommandScheduler.getInstance().run();

        telemetry.addData("speeds", snap.getShootMotor().getVelocity());
        telemetryM.addData("speeeds", snap.getSpeed());
        telemetryM.update();
        telemetry.update();
    }
}
