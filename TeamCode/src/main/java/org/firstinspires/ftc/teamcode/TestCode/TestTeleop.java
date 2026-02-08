package org.firstinspires.ftc.teamcode.TestCode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.ProfiledPIDCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.AreShootersRevedCommand;
import org.firstinspires.ftc.teamcode.Commands.ChassisAutoMoveUsingTime;
import org.firstinspires.ftc.teamcode.Commands.ChassisDefaultFEILDCommand;
import org.firstinspires.ftc.teamcode.Commands.ChassisDefaultFEILDCommandSlowMode;
import org.firstinspires.ftc.teamcode.Commands.ChassisLLAutoDrive;
import org.firstinspires.ftc.teamcode.Commands.ChassisLLAutoTurn;
import org.firstinspires.ftc.teamcode.Commands.ChassisLookToAprilTag;
import org.firstinspires.ftc.teamcode.Commands.ChassisLookToAprilTagInAutoRed;
import org.firstinspires.ftc.teamcode.Commands.CloseShot;
import org.firstinspires.ftc.teamcode.Commands.ControllerFeedback;
import org.firstinspires.ftc.teamcode.Commands.FarShot;
import org.firstinspires.ftc.teamcode.Commands.FeedShooter;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeFeeding;
import org.firstinspires.ftc.teamcode.Commands.LimelightSeesAT;
import org.firstinspires.ftc.teamcode.Commands.RevToVeloUsingPID;
import org.firstinspires.ftc.teamcode.Commands.RevUpToShoot;
import org.firstinspires.ftc.teamcode.Commands.ShootAllThree;
import org.firstinspires.ftc.teamcode.Commands.ShooterDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.SpitCommand;
import org.firstinspires.ftc.teamcode.Commands.Timer;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LLsubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

import java.util.List;

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
    Button resetPos;
    Button rev;
    Button shoot;
    Button lockOn;
    Button spit;


    @Override
    public void initialize(){
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        telemetry.addLine("Init");
        telemetry.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        limelight = new LLsubsystem(hardwareMap);
        gamepad = new GamepadEx(gamepad1);
        //Chassis initialization
        chassis = new ChassisSubsystem(hardwareMap, telemetryM);
        chassisDefault = new ChassisDefaultFEILDCommand(chassis, telemetryM, gamepad);
        //Shooter Initialization
        snap = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.SNAP);
        crackle = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.CRACKLE);
        pop = new ShooterSubsystem(hardwareMap, telemetryM, RobotConstants.Hardware.POP);

        crackleDefault = new ShooterDefaultCommand(crackle);
        popDefault = new ShooterDefaultCommand(pop);
        snapDefault = new ShooterDefaultCommand(snap);
        //Intake Initialization
        intake = new IntakeSubsystem(hardwareMap, telemetryM);
        intakeDefaultCommand = new IntakeDefaultCommand(intake);
        //Trigger Initializations
        intakeTrigger = new Trigger(()->new TriggerReader(gamepad, GamepadKeys.Trigger.LEFT_TRIGGER).isDown());
        holdArms = new Trigger(()->new TriggerReader(gamepad, GamepadKeys.Trigger.RIGHT_TRIGGER).isDown());
        

        //Command based stuff
        register(snap, crackle, pop, chassis, intake);
        schedule(snapDefault, crackleDefault, popDefault, chassisDefault, intakeDefaultCommand);
        chassis.setDefaultCommand(chassisDefault);
        pop.setDefaultCommand(popDefault);
        crackle.setDefaultCommand(crackleDefault);
        snap.setDefaultCommand(snapDefault);
        intake.setDefaultCommand(intakeDefaultCommand);
        resetPos = new GamepadButton(
                gamepad, GamepadKeys.Button.START
        );
        rev= new GamepadButton(
                gamepad, GamepadKeys.Button.RIGHT_BUMPER
        );
        shoot = new GamepadButton(
                gamepad, GamepadKeys.Button.A
        );
        lockOn = new GamepadButton(
                gamepad, GamepadKeys.Button.LEFT_BUMPER
        );
        spit = new GamepadButton(
                gamepad, GamepadKeys.Button.X
        );

        limelight.getLimelight().start();

    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();

        schedule(chassisDefault);
        resetPos.whenPressed(
                new InstantCommand(()->{
                    chassis.resetPos();
        }));
        Button pointAtAT = new GamepadButton(
                gamepad, GamepadKeys.Button.Y
        ).whenPressed(
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new WaitUntilCommand(()->snap.getShooterPID().atSetPoint()&&pop.getShooterPID().atSetPoint()&&crackle.getShooterPID().atSetPoint()),
                                new RevToVeloUsingPID(snap, telemetry),
                                new RevToVeloUsingPID(crackle, telemetry),
                                new RevToVeloUsingPID(pop, telemetry)
                        ),
                        new ParallelRaceGroup(
                                new RevToVeloUsingPID(snap, telemetry),
                                new RevToVeloUsingPID(crackle, telemetry),
                                new RevToVeloUsingPID(pop, telemetry),
                                new ParallelDeadlineGroup(
                                        new Timer(RobotConstants.Teleop.HOLD_THE_ARM),
                                        new FeedShooter(snap)),
                                new ParallelDeadlineGroup(
                                        new Timer(RobotConstants.Teleop.HOLD_THE_ARM),
                                        new FeedShooter(crackle)),
                                new ParallelDeadlineGroup(
                                        new Timer(RobotConstants.Teleop.HOLD_THE_ARM),
                                        new FeedShooter(pop))
                        )


                )



        );
       rev.whileHeld(
                new ParallelCommandGroup(
                        new RevToVeloUsingPID(crackle, telemetry),
                        new RevToVeloUsingPID(snap, telemetry),
                        new RevToVeloUsingPID(pop, telemetry),
                        new ControllerFeedback(gamepad, snap, crackle, pop, telemetry)
                ));


        shoot.whenPressed(
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


        lockOn.whileHeld(
                new ChassisLookToAprilTag(chassis, limelight, telemetryM, 5, gamepad)

        );
        spit.whileHeld(
                new SpitCommand(intake)
        );
        intakeTrigger.whileActiveContinuous(new IntakeCommand(intake, snap, crackle, pop));
        holdArms.whileActiveContinuous(new ParallelCommandGroup(
                new IntakeFeeding(snap),
                new IntakeFeeding(pop)
        ) );

        telemetry.addData("position", chassis.getFollower().getPose());
        telemetry.update();
        telemetryM.update();
    }
}

