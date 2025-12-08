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

import org.firstinspires.ftc.teamcode.Commands.ChassisDefaultFEILDCommand;
import org.firstinspires.ftc.teamcode.Commands.ChassisDefaultROBOTCommand;
import org.firstinspires.ftc.teamcode.Commands.ChassisLookToAprilTag;
import org.firstinspires.ftc.teamcode.Commands.RevUpToShoot;
import org.firstinspires.ftc.teamcode.Commands.Shoot;
import org.firstinspires.ftc.teamcode.Commands.ShootSequence;
import org.firstinspires.ftc.teamcode.Commands.ShooterDefaultCommand;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LLsubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@TeleOp(group = "Test", name = "Hood Test")
public class TestTeleop extends CommandOpMode {
    LLsubsystem limelight;
    ShooterSubsystem snap;
    ChassisSubsystem chassis;
    ChassisDefaultROBOTCommand chassisDefault;
    ShooterDefaultCommand snapDefault;
    GamepadEx gamepad;
    TelemetryManager telemetryM;

    @Override
    public void initialize(){
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        limelight = new LLsubsystem(hardwareMap);
        gamepad = new GamepadEx(gamepad1);
        chassis = new ChassisSubsystem(hardwareMap, telemetryM);
        chassisDefault = new ChassisDefaultROBOTCommand(chassis, telemetryM, gamepad);
        snap = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.SNAP);

        snapDefault = new ShooterDefaultCommand(snap, gamepad);



        register(snap, chassis);
        schedule(snapDefault, chassisDefault);
        chassis.setDefaultCommand(chassisDefault);
        chassis.resetPos();
        CommandScheduler.getInstance().setDefaultCommand(snap, snapDefault);
    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();


        Button fullShoot = new GamepadButton(
                gamepad, GamepadKeys.Button.A
        ).whenPressed(
                new ShootSequence(snap)
        );
        Button shoot = new GamepadButton(
               gamepad, GamepadKeys.Button.RIGHT_BUMPER
       ).whenHeld(
               new RevUpToShoot(snap)
       );
        Button pointAtAT = new GamepadButton(
                gamepad, GamepadKeys.Button.LEFT_BUMPER
        ).whenHeld(
                new ChassisLookToAprilTag(chassis, limelight, telemetryM, 5, gamepad)
        );
        telemetryM.addData("angle", snap.getHoodAngle());
        telemetryM.addData("spped", RobotConstants.Hardware.SHOOTER_WHEEL_GEAR_RATIO);
        telemetryM.addData("ta", snap.getLLResult().getTa());
        telemetryM.update();
        telemetry.update();
    }
}
