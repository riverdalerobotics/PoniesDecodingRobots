package org.firstinspires.ftc.teamcode.TestCode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ChassisDefaultFEILDCommand;
import org.firstinspires.ftc.teamcode.Commands.ChassisLookToAprilTag;
import org.firstinspires.ftc.teamcode.Commands.CloseShot;
import org.firstinspires.ftc.teamcode.Commands.ControllerFeedback;
import org.firstinspires.ftc.teamcode.Commands.FarShot;
import org.firstinspires.ftc.teamcode.Commands.FeedShooter;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeFeeding;
import org.firstinspires.ftc.teamcode.Commands.RevToVeloUsingPID;
import org.firstinspires.ftc.teamcode.Commands.RevUpToShoot;
import org.firstinspires.ftc.teamcode.Commands.ShooterDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.Timer;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LLsubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@TeleOp(group = "Test", name = "Test Teleop")
public class TestTeleop extends CommandOpMode {
    LLsubsystem limelight;
    ShooterSubsystem snap, crackle, pop;
    IntakeSubsystem intake;
    ChassisSubsystem chassis;
    ChassisDefaultFEILDCommand chassisDefault;
    IntakeDefaultCommand intakeDefaultCommand;
    ShooterDefaultCommand snapDefault, crackleDefault, popDefault;
    GamepadEx gamepad;
    TelemetryManager telemetryM;
    Trigger intakeTrigger;
    Trigger holdArms;

    @Override
    public void initialize(){
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        limelight = new LLsubsystem(hardwareMap);
        gamepad = new GamepadEx(gamepad1);

        chassis = new ChassisSubsystem(hardwareMap, telemetryM);
        chassisDefault = new ChassisDefaultFEILDCommand(chassis, telemetryM, gamepad);

        snap = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.SNAP);
        crackle = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.CRACKLE);
        pop = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.POP);

        crackleDefault = new ShooterDefaultCommand(crackle);
        popDefault = new ShooterDefaultCommand(pop);
        snapDefault = new ShooterDefaultCommand(snap);

        intake = new IntakeSubsystem(hardwareMap, telemetryM);
        intakeDefaultCommand = new IntakeDefaultCommand(intake);

        intakeTrigger = new Trigger(()->new TriggerReader(gamepad, GamepadKeys.Trigger.LEFT_TRIGGER).isDown());
        holdArms = new Trigger(()->new TriggerReader(gamepad, GamepadKeys.Trigger.RIGHT_TRIGGER).isDown());
        


        register(snap, crackle, pop, chassis, intake);
        schedule(snapDefault, crackleDefault, popDefault, chassisDefault, intakeDefaultCommand);
        chassis.setDefaultCommand(chassisDefault);
        pop.setDefaultCommand(popDefault);
        crackle.setDefaultCommand(crackleDefault);
        snap.setDefaultCommand(snapDefault);
        intake.setDefaultCommand(intakeDefaultCommand);

    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();


        Button resetPos = new GamepadButton(
                gamepad, GamepadKeys.Button.START
        ).whenPressed(
                new InstantCommand(()->{

                        chassis.resetPos();
        })
        );

        Button rev = new GamepadButton(
               gamepad, GamepadKeys.Button.RIGHT_BUMPER
        ).whileHeld(
                new ParallelCommandGroup(
                        new RevToVeloUsingPID(crackle, telemetry),
                        new RevToVeloUsingPID(snap, telemetry),
                        new RevToVeloUsingPID(pop, telemetry),
                        new ControllerFeedback(gamepad, snap, crackle, pop, telemetry)
                ));


        Button shoot = new GamepadButton(
                gamepad, GamepadKeys.Button.A
        ).whenPressed(
                new ParallelCommandGroup(
                    new ParallelDeadlineGroup(
                        new Timer(RobotConstants.Teleop.HOLD_THE_ARM),
                        new FeedShooter(snap)),
                    new ParallelDeadlineGroup(
                        new Timer(RobotConstants.Teleop.HOLD_THE_ARM),
                        new FeedShooter(crackle)),
                    new ParallelDeadlineGroup(
                            new Timer(RobotConstants.Teleop.HOLD_THE_ARM),
                            new FeedShooter(pop))
        ));


        Button pointAtAT = new GamepadButton(
                gamepad, GamepadKeys.Button.LEFT_BUMPER
        ).whenHeld(
                new ChassisLookToAprilTag(chassis, limelight, telemetryM, 5, gamepad)

        );

        intakeTrigger.whileActiveContinuous(new IntakeCommand(intake, snap, crackle, pop));
        holdArms.whileActiveContinuous(new ParallelCommandGroup(
                new IntakeFeeding(snap),
                new IntakeFeeding(pop)
        ) );

        telemetry.addData("position", chassis.getPose());
        telemetryM.addData("angle", snap.getHoodAngle());
        telemetryM.addData("yaw", chassis.yawPitchRollAngles().getYaw());
        telemetryM.addData("speed", snap.getSpeed());
        telemetryM.addData("ta", snap.getLLResult().getTa());
        telemetryM.update();
    }
}

