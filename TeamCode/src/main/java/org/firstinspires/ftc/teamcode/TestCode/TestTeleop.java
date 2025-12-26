package org.firstinspires.ftc.teamcode.TestCode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ChassisDefaultFEILDCommand;
import org.firstinspires.ftc.teamcode.Commands.ChassisDefaultROBOTCommand;
import org.firstinspires.ftc.teamcode.Commands.ChassisLookToAprilTag;
import org.firstinspires.ftc.teamcode.Commands.CloseShot;
import org.firstinspires.ftc.teamcode.Commands.FarShot;
import org.firstinspires.ftc.teamcode.Commands.FeedShooter;
import org.firstinspires.ftc.teamcode.Commands.RevToVeloUsingPID;
import org.firstinspires.ftc.teamcode.Commands.RevUpToShoot;
import org.firstinspires.ftc.teamcode.Commands.Shoot;
import org.firstinspires.ftc.teamcode.Commands.ShootSequence;
import org.firstinspires.ftc.teamcode.Commands.ShooterDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.Timer;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LLsubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@TeleOp(group = "Test", name = "Test Teleop")
public class TestTeleop extends CommandOpMode {
    LLsubsystem limelight;
    ShooterSubsystem snap, crackle, pop;
    ChassisSubsystem chassis;
    ChassisDefaultFEILDCommand chassisDefault;
    ShooterDefaultCommand snapDefault, crackleDefault, popDefault;
    GamepadEx gamepad;
    CloseShot closeShot;
    FarShot farShot;
    RevUpToShoot revShoot;
    TelemetryManager telemetryM;

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
        snapDefault = new ShooterDefaultCommand(snap);
        crackleDefault = new ShooterDefaultCommand(crackle);
        popDefault = new ShooterDefaultCommand(pop);
        snapDefault = new ShooterDefaultCommand(snap);
        revShoot = new RevUpToShoot(snap);
        closeShot = new CloseShot(snap);
        farShot = new FarShot(snap);
        


        register(snap, crackle, pop, chassis);
        schedule(snapDefault, crackleDefault, popDefault, chassisDefault);
        chassis.setDefaultCommand(chassisDefault);
        pop.setDefaultCommand(popDefault);
        crackle.setDefaultCommand(crackleDefault);
        CommandScheduler.getInstance().setDefaultCommand(snap, snapDefault);
    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();


        if(gamepad.isDown(GamepadKeys.Button.START)){
            chassis.resetPos();
        }
        Button rev = new GamepadButton(
               gamepad, GamepadKeys.Button.RIGHT_BUMPER
        ).whileHeld(


                       new RevToVeloUsingPID(crackle, RobotConstants.Teleop.FAR_SHOT, telemetry)


        ).whileHeld(new RevToVeloUsingPID(snap, RobotConstants.Teleop.FAR_SHOT, telemetry)).
            whileHeld(new RevToVeloUsingPID(pop, RobotConstants.Teleop.FAR_SHOT, telemetry));
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
        if(gamepad1.x){
            gamepad.gamepad.rumble(500);
        }
        telemetry.addData("speed", snap.getSpeed());
        telemetry.addData("SET SPEED", snap.setSpeed);
        telemetryM.addData("angle", snap.getHoodAngle());
        telemetryM.addData("yaw", chassis.yawPitchRollAngles().getYaw());
        telemetryM.addData("speed", snap.getSpeed());
        telemetryM.addData("ta", snap.getLLResult().getTa());
        telemetryM.update();
    }
}
