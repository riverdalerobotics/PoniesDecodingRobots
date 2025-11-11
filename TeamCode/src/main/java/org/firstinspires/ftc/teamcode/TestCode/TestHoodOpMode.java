package org.firstinspires.ftc.teamcode.TestCode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.gamepad.Gamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.HoodTestCommand;
import org.firstinspires.ftc.teamcode.Commands.ShooterDefaultCommand;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@TeleOp(group = "Test", name = "Hood Test")
public class TestHoodOpMode extends CommandOpMode {
    ShooterSubsystem snap;
    ShooterDefaultCommand snapDefault;
    GamepadEx gamepad;
    TelemetryManager telemetryM;

    @Override
    public void initialize(){
        gamepad = new GamepadEx(gamepad1);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        snap = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.SNAP);

        snapDefault = new ShooterDefaultCommand(snap, gamepad);


        register(snap);
        schedule(snapDefault);

        CommandScheduler.getInstance().setDefaultCommand(snap, snapDefault);
    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();
        if(gamepad1.a){
            snap.setHoodAngle(0);
        }
        if(gamepad1.b){
            snap.setHoodAngle(0.05);
        }
        if(gamepad1.y){
            snap.setHoodAngle(0.1);
        }
        telemetryM.addData("angle", snap.getHoodAngle());
        telemetryM.addData("ta", snap.getLLResult().getTa());
        telemetryM.update();
        telemetry.update();
    }
}
